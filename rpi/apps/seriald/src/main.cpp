#include <atomic>
#include <mutex>
#include <poll.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "async_logger.h"
#include "sink_file.h"
#include "sink_stdout.h"
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>
#include <mc/serial/Uart.hpp>

static constexpr int DEFAULT_BAUD = 921600;
static const char *DEFAULT_DEV = "/dev/serial0";
static const char *DEFAULT_SOCK = "/run/roboracer/seriald.sock";
static const char *DEFAULT_LOG = "/var/log/roboracer/seriald.log";

#pragma pack(push, 1)
struct EspLogPayload {
	uint8_t level;
	uint8_t category;
	uint16_t code_le;
	uint32_t t_ms_le;
	int32_t a0_le;
	int32_t a1_le;
	int32_t a2_le;
	int32_t a3_le;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct EspStatusPayload {
	uint16_t last_seq_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t ttl_ms_le;
	uint16_t dist_mm_le;
	uint16_t faults_le;
	uint16_t rx_bad_crc_le;
	uint16_t rx_bad_cobs_le;
};
#pragma pack(pop)

static inline uint16_t rd16u(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t rd16s(const uint8_t *p) { return (int16_t)rd16u(p); }
static inline uint32_t rd32u(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
		   ((uint32_t)p[3] << 24);
}
static inline int32_t rd32s(const uint8_t *p) { return (int32_t)rd32u(p); }

static LogLevel mapEspLv(uint8_t lv) {
	if (lv > 5)
		lv = 5;
	return (LogLevel)lv;
}

struct TxMsg {
	uint16_t len;
	uint8_t data[mc::proto::MAX_FRAME_ENCODED];
};

static std::mutex g_tx_mu;
static std::queue< TxMsg > g_tx_q;

static void tx_enqueue(const uint8_t *data, uint16_t len) {
	if (len == 0 || len > sizeof(TxMsg::data))
		return;
	std::lock_guard< std::mutex > lk(g_tx_mu);
	TxMsg m{};
	m.len = len;
	memcpy(m.data, data, len);
	g_tx_q.push(m);
}

static bool tx_dequeue(TxMsg &out) {
	std::lock_guard< std::mutex > lk(g_tx_mu);
	if (g_tx_q.empty())
		return false;
	out = g_tx_q.front();
	g_tx_q.pop();
	return true;
}

int main(int argc, char **argv) {
	std::string dev = DEFAULT_DEV;
	int baud = DEFAULT_BAUD;
	std::string sock = DEFAULT_SOCK;
	std::string logpath = DEFAULT_LOG;

	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if (a == "--dev" && i + 1 < argc)
			dev = argv[++i];
		else if (a == "--baud" && i + 1 < argc)
			baud = atoi(argv[++i]);
		else if (a == "--sock" && i + 1 < argc)
			sock = argv[++i];
		else if (a == "--log" && i + 1 < argc)
			logpath = argv[++i];
	}

	AsyncLogger log;
	log.addSink(std::make_unique< StdoutSink >());
	log.addSink(std::make_unique< FileSink >(logpath));
	log.start();

	log.log(LogLevel::INFO, "seriald starting dev=" + dev + " baud=" +
								std::to_string(baud) + " sock=" + sock);

	mc::serial::Uart uart;
	if (!uart.open(dev, baud)) {
		log.log(LogLevel::FATAL, "failed to open uart " + dev);
		return 1;
	}

	mc::ipc::UdsServer ipc;
	if (!ipc.listen(sock)) {
		log.log(LogLevel::FATAL, "failed to open ipc " + sock);
		return 1;
	}

	mc::proto::PacketReader pr;

	std::atomic< bool > run{true};

	std::thread txThread([&] {
		while (run) {
			TxMsg m{};
			if (tx_dequeue(m)) {
				int off = 0;
				while (off < (int)m.len) {
					int w = uart.write(m.data + off, (int)m.len - off);
					if (w < 0) {
						log.log(LogLevel::ERROR, "uart write failed");
						break;
					}
					if (w == 0)
						break;
					off += w;
				}
			} else {
				usleep(1000);
			}
		}
	});

	while (run) {
		std::vector< pollfd > fds;
		fds.reserve(2 + ipc.clients().size());

		pollfd uart_fd{};
		uart_fd.fd = uart.fd();
		uart_fd.events = POLLIN;
		fds.push_back(uart_fd);

		pollfd srv_fd{};
		srv_fd.fd = ipc.fd();
		srv_fd.events = POLLIN;
		fds.push_back(srv_fd);

		for (int cfd : ipc.clients()) {
			pollfd p{};
			p.fd = cfd;
			p.events = POLLIN;
			fds.push_back(p);
		}

		int r = ::poll(fds.data(), (nfds_t)fds.size(), 10);
		if (r < 0)
			continue;

		if (fds[1].revents & POLLIN) {
			while (true) {
				int cfd = ipc.accept_client();
				if (cfd < 0)
					break;
			}
		}

		if (fds[0].revents & POLLIN) {
			uint8_t buf[1024];
			int n = uart.read(buf, (int)sizeof(buf));
			if (n > 0) {
				for (int i = 0; i < n; i++) {
					if (pr.push(buf[i]) && pr.hasFrame()) {
						const auto &f = pr.frame();

						if (f.hdr.type == (uint8_t)mc::proto::Type::LOG &&
							f.payload_len == sizeof(EspLogPayload)) {
							const uint8_t *p = f.payload;
							uint8_t lv = p[0];
							uint8_t cat = p[1];
							uint16_t code = rd16u(p + 2);
							uint32_t tms = rd32u(p + 4);
							int32_t a0 = rd32s(p + 8);
							int32_t a1 = rd32s(p + 12);
							int32_t a2 = rd32s(p + 16);
							int32_t a3 = rd32s(p + 20);

							log.log(mapEspLv(lv),
									"ESPLOG cat=" + std::to_string(cat) +
										" code=" + std::to_string(code) +
										" t_ms=" + std::to_string(tms) +
										" a0=" + std::to_string(a0) +
										" a1=" + std::to_string(a1) +
										" a2=" + std::to_string(a2) +
										" a3=" + std::to_string(a3));
						} else if (f.hdr.type ==
									   (uint8_t)mc::proto::Type::STATUS &&
								   f.payload_len == sizeof(EspStatusPayload)) {
							const uint8_t *p = f.payload;
							uint16_t last_seq = rd16u(p + 0);
							int16_t vmm = rd16s(p + 2);
							int16_t scd = rd16s(p + 4);
							uint16_t ttl = rd16u(p + 6);
							uint16_t dist = rd16u(p + 8);
							uint16_t faults = rd16u(p + 10);
							uint16_t badcrc = rd16u(p + 12);
							uint16_t badcobs = rd16u(p + 14);

							log.log(
								LogLevel::INFO,
								"STATUS seq=" + std::to_string(last_seq) +
									" v_mm_s=" + std::to_string(vmm) +
									" steer_cdeg=" + std::to_string(scd) +
									" ttl_ms=" + std::to_string(ttl) +
									" dist_mm=" + std::to_string(dist) +
									" faults=0x" +
									[](uint16_t x) {
										char b[16];
										snprintf(b, sizeof(b), "%04x", x);
										return std::string(b);
									}(faults) +
									" rx_bad_crc=" + std::to_string(badcrc) +
									" rx_bad_cobs=" + std::to_string(badcobs));
						} else {
							log.log(
								LogLevel::DEBUG,
								"RX type=0x" +
									[](uint8_t x) {
										char b[8];
										snprintf(b, sizeof(b), "%02x", x);
										return std::string(b);
									}(f.hdr.type) +
									" len=" + std::to_string(f.payload_len));
						}

						uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
						size_t enc_len = 0;
						const uint16_t seq = mc::proto::from_le16(f.hdr.seq_le);
						if (mc::proto::PacketWriter::build(
								enc, sizeof(enc), enc_len,
								static_cast< mc::proto::Type >(f.hdr.type),
								f.hdr.flags, seq, f.payload, f.payload_len)) {
							for (int cfd : ipc.clients()) {
								(void)::send(cfd, enc, enc_len, MSG_NOSIGNAL);
							}
						}

						pr.consumeFrame();
					}
				}
			}
		}

		for (size_t i = 2; i < fds.size(); ++i) {
			if (!(fds[i].revents & POLLIN))
				continue;
			uint8_t buf[mc::proto::MAX_FRAME_ENCODED];
			ssize_t n = ::recv(fds[i].fd, buf, sizeof(buf), 0);
			if (n <= 0) {
				ipc.remove_client(fds[i].fd);
				continue;
			}
			tx_enqueue(buf, (uint16_t)n);
			log.log(LogLevel::DEBUG,
					"IPC->UART forward bytes=" + std::to_string(n));
		}
	}

	run = false;
	if (txThread.joinable())
		txThread.join();
	ipc.close();
	uart.close();
	log.stop();
	return 0;
}
