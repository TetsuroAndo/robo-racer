#include <atomic>
#include <errno.h>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "../config/Config.h"
#include <mc/core/Log.hpp>
#include <mc/core/Path.hpp>
#include <mc/core/Time.hpp>
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
	uint8_t applied_brake_duty;
	uint8_t stop_level;
	uint8_t stop_requested;
	uint8_t reserved;
};
#pragma pack(pop)

static inline uint16_t rd16u(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t rd16s(const uint8_t *p) { return (int16_t)rd16u(p); }
static mc::core::LogLevel mapEspLv(uint8_t lv) {
	if (lv > 5)
		lv = 5;
	return (mc::core::LogLevel)lv;
}

static bool starts_with(const std::string &s, const char *prefix) {
	const size_t n = ::strlen(prefix);
	return s.size() >= n && s.compare(0, n, prefix) == 0;
}

struct TxMsg {
	uint16_t len;
	uint8_t data[mc::proto::MAX_FRAME_ENCODED];
};

struct ServerEntry {
	mc::ipc::UdsServer *server;
	bool telemetry;
	std::string role;
};

struct TcpClientEntry {
	int fd;
};

static int g_tcp_listen_fd = -1;
static std::vector< TcpClientEntry > g_tcp_clients;

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

static void add_server(std::vector< ServerEntry > &servers,
					   mc::ipc::UdsServer &server, const std::string &path,
					   bool telemetry, const std::string &role,
					   mc::core::Logger &logger) {
	if (path.empty())
		return;
	mc::core::ensure_dir(mc::core::dir_of(path));
	if (!server.listen(path)) {
		logger.log(mc::core::LogLevel::Error, "seriald",
				   "failed to open ipc " + role + " path=" + path);
		return;
	}
	servers.push_back(ServerEntry{&server, telemetry, role});
}

static void
remove_client_from_servers(const std::vector< ServerEntry > &servers, int cfd) {
	for (const auto &entry : servers) {
		entry.server->remove_client(cfd);
	}
}

static void broadcast_to(const std::vector< ServerEntry > &servers,
						 bool telemetry, const uint8_t *data, size_t len) {
	for (const auto &entry : servers) {
		if (entry.telemetry != telemetry)
			continue;
		const std::vector< int > clients = entry.server->clients();
		for (int cfd : clients) {
			const ssize_t w = ::send(cfd, data, len, MSG_NOSIGNAL);
			if (w == (ssize_t)len)
				continue;
			if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
				continue;
			entry.server->remove_client(cfd);
		}
	}
}

static bool setup_tcp_listener(int port, mc::core::Logger &logger) {
	if (port <= 0)
		return false;
	int fd = ::socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		logger.log(mc::core::LogLevel::Error, "seriald",
				   "failed to create telemetry tcp socket");
		return false;
	}
	int fl = fcntl(fd, F_GETFL, 0);
	if (fl >= 0) {
		(void)fcntl(fd, F_SETFL, fl | O_NONBLOCK);
	}
	int one = 1;
	::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(static_cast< uint16_t >(port));

	if (::bind(fd, reinterpret_cast< sockaddr * >(&addr), sizeof(addr)) != 0) {
		logger.log(mc::core::LogLevel::Error, "seriald",
				   "failed to bind telemetry tcp port=" + std::to_string(port));
		::close(fd);
		return false;
	}
	if (::listen(fd, 16) != 0) {
		logger.log(mc::core::LogLevel::Error, "seriald",
				   "failed to listen telemetry tcp port=" +
					   std::to_string(port));
		::close(fd);
		return false;
	}
	g_tcp_listen_fd = fd;
	logger.log(mc::core::LogLevel::Info, "seriald",
			   "telemetry tcp listen port=" + std::to_string(port));
	return true;
}

static void tcp_accept_clients(mc::core::Logger &logger) {
	if (g_tcp_listen_fd < 0)
		return;
	while (true) {
		int cfd = ::accept(g_tcp_listen_fd, nullptr, nullptr);
		if (cfd < 0)
			break;
		int fl = fcntl(cfd, F_GETFL, 0);
		if (fl >= 0) {
			(void)fcntl(cfd, F_SETFL, fl | O_NONBLOCK);
		}
		g_tcp_clients.push_back(TcpClientEntry{cfd});
		logger.log(mc::core::LogLevel::Info, "seriald",
				   "tcp telemetry client connected fd=" + std::to_string(cfd));
	}
}

static void tcp_remove_client(size_t idx) {
	if (idx >= g_tcp_clients.size())
		return;
	::close(g_tcp_clients[idx].fd);
	g_tcp_clients.erase(g_tcp_clients.begin() + (long)idx);
}

static void tcp_broadcast(const uint8_t *data, size_t len) {
	for (size_t i = 0; i < g_tcp_clients.size();) {
		int fd = g_tcp_clients[i].fd;
		ssize_t w = ::send(fd, data, len, MSG_NOSIGNAL);
		if (w == (ssize_t)len) {
			++i;
			continue;
		}
		if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			++i;
			continue;
		}
		if (w < 0 || (size_t)w != len) {
			tcp_remove_client(i);
			continue;
		}
		++i;
	}
}

static void tcp_close_all() {
	for (const auto &c : g_tcp_clients) {
		::close(c.fd);
	}
	g_tcp_clients.clear();
	if (g_tcp_listen_fd >= 0) {
		::close(g_tcp_listen_fd);
		g_tcp_listen_fd = -1;
	}
}

int main(int argc, char **argv) {
	std::string dev = seriald_cfg::DEFAULT_DEV;
	int baud = seriald_cfg::DEFAULT_BAUD;
	std::string control_sock = seriald_cfg::DEFAULT_CONTROL_SOCK;
	std::string telemetry_sock = seriald_cfg::DEFAULT_TELEMETRY_SOCK;
	int telemetry_tcp_port = seriald_cfg::DEFAULT_TELEMETRY_TCP_PORT;
	std::string logpath = seriald_cfg::DEFAULT_LOG;
	std::string compat_control_sock = seriald_cfg::DEFAULT_COMPAT_CONTROL_SOCK;
	std::string compat_telemetry_sock =
		seriald_cfg::DEFAULT_COMPAT_TELEMETRY_SOCK;

	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if (a == "--dev" && i + 1 < argc)
			dev = argv[++i];
		else if (a == "--baud" && i + 1 < argc)
			baud = atoi(argv[++i]);
		else if ((a == "--sock" || a == "--control-sock") && i + 1 < argc)
			control_sock = argv[++i];
		else if (a == "--telemetry-sock" && i + 1 < argc)
			telemetry_sock = argv[++i];
		else if (a == "--telemetry-tcp-port" && i + 1 < argc)
			telemetry_tcp_port = atoi(argv[++i]);
		else if (a == "--log" && i + 1 < argc)
			logpath = argv[++i];
	}

	mc::core::ensure_dir(mc::core::dir_of(logpath));

	auto &logger = mc::core::Logger::instance();
	logger.addSink(std::make_shared< mc::core::FileSink >(logpath));

	logger.log(mc::core::LogLevel::Info, "seriald",
			   "starting dev=" + dev + " baud=" + std::to_string(baud) +
				   " control_sock=" + control_sock +
				   " telemetry_sock=" + telemetry_sock +
				   " telemetry_tcp_port=" + std::to_string(telemetry_tcp_port));

	mc::serial::Uart uart;
	if (!uart.open(dev, baud)) {
		logger.log(mc::core::LogLevel::Fatal, "seriald",
				   "failed to open uart " + dev);
		return 1;
	}

	mc::ipc::UdsServer control_ipc(SOCK_SEQPACKET);
	mc::ipc::UdsServer control_compat_ipc(SOCK_SEQPACKET);
	mc::ipc::UdsServer telemetry_ipc(SOCK_SEQPACKET);
	mc::ipc::UdsServer telemetry_compat_ipc(SOCK_SEQPACKET);

	std::vector< ServerEntry > servers;
	add_server(servers, control_ipc, control_sock, false, "control", logger);
	if (compat_control_sock != control_sock) {
		add_server(servers, control_compat_ipc, compat_control_sock, false,
				   "control_compat", logger);
	}
	add_server(servers, telemetry_ipc, telemetry_sock, true, "telemetry",
			   logger);
	if (compat_telemetry_sock != telemetry_sock) {
		add_server(servers, telemetry_compat_ipc, compat_telemetry_sock, true,
				   "telemetry_compat", logger);
	}
	bool have_control = false;
	bool have_telemetry = false;
	for (const auto &entry : servers) {
		have_control = have_control || !entry.telemetry;
		have_telemetry = have_telemetry || entry.telemetry;
	}
	if (!have_control || !have_telemetry) {
		logger.log(mc::core::LogLevel::Fatal, "seriald",
				   "failed to open required ipc sockets");
		return 1;
	}
	if (telemetry_tcp_port > 0) {
		setup_tcp_listener(telemetry_tcp_port, logger);
	}

	mc::proto::PacketReader pr;
	uint32_t last_imu_log_ms = 0;

	std::atomic< bool > run{true};
	std::thread txThread([&] {
		while (run) {
			TxMsg m{};
			if (tx_dequeue(m)) {
				int off = 0;
				while (off < (int)m.len) {
					int w = uart.write(m.data + off, (int)m.len - off);
					if (w < 0) {
						logger.log(mc::core::LogLevel::Error, "seriald",
								   "uart write failed");
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
		fds.reserve(2 + servers.size() + seriald_cfg::MAX_CLIENT_FDS);

		pollfd uart_fd{};
		uart_fd.fd = uart.fd();
		uart_fd.events = POLLIN;
		fds.push_back(uart_fd);

		for (const auto &entry : servers) {
			pollfd srv_fd{};
			srv_fd.fd = entry.server->fd();
			srv_fd.events = POLLIN;
			fds.push_back(srv_fd);
		}

		if (g_tcp_listen_fd >= 0) {
			pollfd tcp_fd{};
			tcp_fd.fd = g_tcp_listen_fd;
			tcp_fd.events = POLLIN;
			fds.push_back(tcp_fd);
		}

		struct ClientEntry {
			int fd;
			bool telemetry;
		};
		std::vector< ClientEntry > client_entries;
		for (const auto &entry : servers) {
			const auto &clients = entry.server->clients();
			size_t client_count = clients.size();
			if (client_count > (size_t)seriald_cfg::MAX_CLIENT_FDS)
				client_count = seriald_cfg::MAX_CLIENT_FDS;
			for (size_t i = 0; i < client_count; ++i) {
				int cfd = clients[i];
				client_entries.push_back(ClientEntry{cfd, entry.telemetry});
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

		const size_t server_start = 1;
		for (size_t i = 0; i < servers.size(); ++i) {
			if (!(fds[server_start + i].revents & POLLIN))
				continue;
			while (true) {
				int cfd = servers[i].server->accept_client();
				if (cfd < 0)
					break;
				logger.log(mc::core::LogLevel::Info, "seriald",
						   "client connected role=" + servers[i].role +
							   " fd=" + std::to_string(cfd));
			}
		}

		size_t tcp_poll_idx = server_start + servers.size();
		if (g_tcp_listen_fd >= 0) {
			if (fds[tcp_poll_idx].revents & POLLIN) {
				tcp_accept_clients(logger);
			}
			++tcp_poll_idx;
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
							const std::string tag =
								starts_with(msg, "tsd20") ? "tsd20" : "esp32";
							logger.log(mapEspLv(lv), tag, msg);
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
							uint8_t applied_brake_duty = p[10];
							uint8_t stop_level = p[11];
							uint8_t stop_requested = p[12];

							logger.log(
								mc::core::LogLevel::Info, "seriald",
								"STATUS seq=" + std::to_string(seq) +
									" auto=" + std::to_string(auto_active) +
									" v_mm_s=" + std::to_string(vmm) +
									" steer_cdeg=" + std::to_string(scd) +
									" age_ms=" + std::to_string(age_ms) +
									" faults=0x" +
									[](uint16_t x) {
										char b[16];
										snprintf(b, sizeof(b), "%04x", x);
										return std::string(b);
									}(faults) +
									" brake_duty=" +
									std::to_string(applied_brake_duty) +
									" stop_level=" +
									std::to_string(stop_level) + " stop_req=" +
									std::to_string(stop_requested));
						} else if (f.type() ==
									   (uint8_t)mc::proto::Type::IMU_STATUS &&
								   f.payload_len ==
									   sizeof(mc::proto::ImuStatusPayload)) {
							const uint8_t *p = f.payload;
							const int16_t a_long = rd16s(p + 0);
							const int16_t v_est = rd16s(p + 2);
							const uint16_t a_cap = rd16u(p + 4);
							const int16_t yaw_x10 = rd16s(p + 6);
							const uint16_t age_ms = rd16u(p + 8);
							const uint8_t flags = p[10];
							const uint8_t reserved = p[11];
							(void)reserved;
							const bool valid = (flags & (1u << 0)) != 0;
							const bool calibrated = (flags & (1u << 1)) != 0;
							const bool brake_mode = (flags & (1u << 2)) != 0;
							const uint32_t now_ms = mc::core::Time::ms();
							if (now_ms - last_imu_log_ms >=
								seriald_cfg::IMU_LOG_INTERVAL_MS) {
								last_imu_log_ms = now_ms;
								logger.log(
									mc::core::LogLevel::Info, "imu",
									"valid=" + std::to_string(valid) +
										" calib=" + std::to_string(calibrated) +
										" brake_mode=" +
										std::to_string(brake_mode) +
										" a_long_mm_s2=" +
										std::to_string(a_long) +
										" v_est_mm_s=" + std::to_string(v_est) +
										" yaw_dps=" +
										std::to_string(
											static_cast< float >(yaw_x10) *
											0.1f) +
										" a_brake_cap_mm_s2=" +
										std::to_string(a_cap) +
										" age_ms=" + std::to_string(age_ms));
							}
						} else if (f.type() ==
									   (uint8_t)mc::proto::Type::TSD20_STATUS &&
								   f.payload_len ==
									   sizeof(mc::proto::Tsd20StatusPayload)) {
							const uint8_t *p = f.payload;
							const uint16_t mm = rd16u(p + 0);
							const uint16_t period_ms = rd16u(p + 2);
							const uint16_t age_ms = rd16u(p + 4);
							const uint8_t fails = p[6];
							const uint8_t flags = p[7];
							const bool ready = (flags & (1u << 0)) != 0;
							const bool valid = (flags & (1u << 1)) != 0;
							logger.log(
								mc::core::LogLevel::Info, "tsd20",
								"ready=" + std::to_string(ready) +
									" valid=" + std::to_string(valid) +
									" mm=" + std::to_string(mm) +
									" fails=" + std::to_string(fails) +
									" period_ms=" + std::to_string(period_ms) +
									" age_ms=" + std::to_string(age_ms));
						} else {
							logger.log(
								mc::core::LogLevel::Debug, "seriald",
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
							broadcast_to(servers, false, enc, enc_len);
							broadcast_to(servers, true, enc, enc_len);
							tcp_broadcast(enc, enc_len);
						}

						pr.consumeFrame();
					}
				}
			}
		}

		const size_t client_start = tcp_poll_idx;
		for (size_t i = client_start; i < fds.size(); ++i) {
			if (!(fds[i].revents & POLLIN))
				continue;
			const ClientEntry &entry = client_entries[i - client_start];
			uint8_t buf[mc::proto::MAX_FRAME_ENCODED];
			ssize_t n = ::recv(fds[i].fd, buf, sizeof(buf), 0);
			if (n <= 0) {
				remove_client_from_servers(servers, fds[i].fd);
				continue;
			}
			if (entry.telemetry) {
				logger.log(mc::core::LogLevel::Warn, "seriald",
						   "telemetry client attempted send; disconnect fd=" +
							   std::to_string(fds[i].fd));
				remove_client_from_servers(servers, fds[i].fd);
				continue;
			}
			tx_enqueue(buf, (uint16_t)n);
			broadcast_to(servers, true, buf, (size_t)n);
			tcp_broadcast(buf, (size_t)n);
			logger.log(mc::core::LogLevel::Debug, "seriald",
					   "IPC->UART forward bytes=" + std::to_string(n));
		}

		for (size_t i = 0; i < g_tcp_clients.size();) {
			pollfd tcp_client{};
			tcp_client.fd = g_tcp_clients[i].fd;
			tcp_client.events = POLLIN;
			int rcv = ::poll(&tcp_client, 1, 0);
			if (rcv > 0 && (tcp_client.revents & POLLIN)) {
				uint8_t dummy[64];
				(void)::recv(g_tcp_clients[i].fd, dummy, sizeof(dummy), 0);
				logger.log(
					mc::core::LogLevel::Warn, "seriald",
					"tcp telemetry client attempted send; disconnect fd=" +
						std::to_string(g_tcp_clients[i].fd));
				tcp_remove_client(i);
				continue;
			}
			++i;
		}
	}

	run = false;
	if (txThread.joinable())
		txThread.join();
	tcp_close_all();
	uart.close();
	logger.shutdown();
	return 0;
}
