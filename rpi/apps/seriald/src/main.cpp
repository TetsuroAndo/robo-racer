#include <atomic>
#include <mutex>
#include <poll.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "../config/Config.h"
#include "async_logger.h"
#include "sink_file.h"
#include "sink_stdout.h"
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>
#include <mc/serial/Uart.hpp>

#pragma pack(push, 1)
struct EspStatusPayload {
	uint8_t seq_applied;
	uint8_t auto_active;
	uint16_t faults_le;
	int16_t speed_mm_s_le;
	int16_t steer_cdeg_le;
	uint16_t age_ms_le;
};
#pragma pack(pop)

static inline uint16_t rd16u(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t rd16s(const uint8_t *p) { return (int16_t)rd16u(p); }
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

static void ensure_dir_(const std::string &path) {
	if (path.empty())
		return;
	const int rc = mkdir(path.c_str(), 0755);
	if (rc == 0 || errno == EEXIST)
		return;
}

static std::string dir_of_(const std::string &path) {
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return std::string();
	return path.substr(0, pos);
}

int main(int argc, char **argv) {
	std::string dev = seriald_cfg::DEFAULT_DEV;
	int baud = seriald_cfg::DEFAULT_BAUD;
	std::string sock = seriald_cfg::DEFAULT_SOCK;
	std::string logpath = seriald_cfg::DEFAULT_LOG;
#ifdef __APPLE__
	bool ipc_dgram = true;
#else
	bool ipc_dgram = false;
#endif

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

	ensure_dir_(dir_of_(sock));
	ensure_dir_(dir_of_(logpath));

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

	mc::ipc::UdsServer ipc(ipc_dgram ? SOCK_DGRAM : SOCK_SEQPACKET);
	if (!ipc.listen(sock)) {
		log.log(LogLevel::FATAL, "failed to open ipc " + sock);
		return 1;
	}

	mc::proto::PacketReader pr;

	std::atomic< bool > run{true};
	struct DgramClient {
		sockaddr_un addr{};
		socklen_t len = 0;
	};
	std::vector< DgramClient > dgram_clients;

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
				usleep(seriald_cfg::TX_IDLE_US);
			}
		}
	});

	while (run) {
		std::vector< pollfd > fds;
		fds.reserve(2 + seriald_cfg::MAX_CLIENT_FDS);

		pollfd uart_fd{};
		uart_fd.fd = uart.fd();
		uart_fd.events = POLLIN;
		fds.push_back(uart_fd);

		pollfd srv_fd{};
		srv_fd.fd = ipc.fd();
		srv_fd.events = POLLIN;
		fds.push_back(srv_fd);

		if (!ipc_dgram) {
			const auto &clients = ipc.clients();
			size_t client_count = clients.size();
			if (client_count > (size_t)seriald_cfg::MAX_CLIENT_FDS)
				client_count = seriald_cfg::MAX_CLIENT_FDS;

			for (size_t i = 0; i < client_count; ++i) {
				int cfd = clients[i];
				pollfd p{};
				p.fd = cfd;
				p.events = POLLIN;
				fds.push_back(p);
			}
		}

		int r = ::poll(fds.data(), (nfds_t)fds.size(),
					   seriald_cfg::POLL_TIMEOUT_MS);
		if (r < 0)
			continue;

		if (fds[1].revents & POLLIN) {
			if (!ipc_dgram) {
				while (true) {
					int cfd = ipc.accept_client();
					if (cfd < 0)
						break;
				}
			} else {
				uint8_t buf[mc::proto::MAX_FRAME_ENCODED];
				sockaddr_un addr{};
				socklen_t alen = sizeof(addr);
				ssize_t n = ::recvfrom(ipc.fd(), buf, sizeof(buf), 0,
									   (sockaddr *)&addr, &alen);
				if (n > 0) {
					bool known = false;
					for (const auto &c : dgram_clients) {
						if (c.len == alen &&
							strncmp(c.addr.sun_path, addr.sun_path,
									sizeof(addr.sun_path)) == 0) {
							known = true;
							break;
						}
					}
					if (!known) {
						DgramClient c{};
						c.addr = addr;
						c.len = alen;
						dgram_clients.push_back(c);
					}
					tx_enqueue(buf, (uint16_t)n);
					log.log(LogLevel::DEBUG, "IPC(dgram)->UART forward bytes=" +
												 std::to_string(n));
				}
			}
		}

		if (fds[0].revents & POLLIN) {
			uint8_t buf[1024];
			int n = uart.read(buf, (int)sizeof(buf));
			if (n > 0) {
				for (int i = 0; i < n; i++) {
					if (pr.push(buf[i]) && pr.hasFrame()) {
						const auto &f = pr.frame();

						if (f.type() == (uint8_t)mc::proto::Type::LOG &&
							f.payload_len >= 1) {
							const uint8_t *p = f.payload;
							uint8_t lv = p[0];
							std::string msg;
							if (f.payload_len > 1) {
								msg.assign(
									reinterpret_cast< const char * >(p + 1),
									f.payload_len - 1);
							}
							if (msg.empty()) {
								msg = "ESPLOG (empty)";
							}
							log.log(mapEspLv(lv), msg);
						} else if (f.type() ==
									   (uint8_t)mc::proto::Type::STATUS &&
								   f.payload_len == sizeof(EspStatusPayload)) {
							const uint8_t *p = f.payload;
							uint8_t seq = p[0];
							uint8_t auto_active = p[1];
							uint16_t faults = rd16u(p + 2);
							int16_t vmm = rd16s(p + 4);
							int16_t scd = rd16s(p + 6);
							uint16_t age_ms = rd16u(p + 8);

							log.log(
								LogLevel::INFO,
								"STATUS seq=" + std::to_string(seq) +
									" auto=" + std::to_string(auto_active) +
									" v_mm_s=" + std::to_string(vmm) +
									" steer_cdeg=" + std::to_string(scd) +
									" age_ms=" + std::to_string(age_ms) +
									" faults=0x" + [](uint16_t x) {
										char b[16];
										snprintf(b, sizeof(b), "%04x", x);
										return std::string(b);
									}(faults));
						} else {
							log.log(
								LogLevel::DEBUG,
								"RX type=0x" +
									[](uint8_t x) {
										char b[8];
										snprintf(b, sizeof(b), "%02x", x);
										return std::string(b);
									}(f.type()) +
									" len=" + std::to_string(f.payload_len));
						}

						uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
						size_t enc_len = 0;
						const uint16_t seq = f.seq();
						if (mc::proto::PacketWriter::build(
								enc, sizeof(enc), enc_len,
								static_cast< mc::proto::Type >(f.type()),
								f.flags(), seq, f.payload, f.payload_len)) {
							if (!ipc_dgram) {
								for (int cfd : ipc.clients()) {
									(void)::send(cfd, enc, enc_len,
												 MSG_NOSIGNAL);
								}
							} else {
								for (const auto &c : dgram_clients) {
									(void)::sendto(ipc.fd(), enc, enc_len, 0,
												   (const sockaddr *)&c.addr,
												   c.len);
								}
							}
						}

						pr.consumeFrame();
					}
				}
			}
		}

		if (!ipc_dgram) {
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
	}

	run = false;
	if (txThread.joinable())
		txThread.join();
	ipc.close();
	uart.close();
	log.stop();
	return 0;
}
