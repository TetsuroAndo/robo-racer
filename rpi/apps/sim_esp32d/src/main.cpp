#include "../config/Config.h"
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <poll.h>
#include <unistd.h>

#include <mc/core/Log.hpp>
#include <mc/core/Path.hpp>
#include <mc/core/Time.hpp>
#include <mc/proto/Proto.hpp>
#include <mc/serial/Uart.hpp>

namespace {

static std::atomic< bool > g_run{true};

void on_sigint(int) { g_run.store(false); }

static inline uint16_t rd16u(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t rd16s(const uint8_t *p) { return (int16_t)rd16u(p); }
static inline void wr16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)(v >> 8);
}

struct SimState {
	bool auto_mode = false;
	bool kill_latched = false;
	bool has_drive = false;
	uint32_t status_seq = 0;
	uint16_t last_drive_seq = 0;
	uint32_t last_drive_ms = 0;
	uint32_t last_ping_ms = 0;
	int16_t target_steer_cdeg = 0;
	int16_t target_speed_mm_s = 0;
	uint16_t target_ttl_ms = 0;
};

struct Config {
	std::string dev;
	std::string log_path;
	int baud = 921600;
	uint32_t status_interval_ms = 50;
	uint32_t status_delay_ms = 0;
	uint32_t status_drop_every = 0;
	uint32_t status_corrupt_every = 0;
	uint32_t hb_timeout_ms = 200;
};

bool send_status(mc::serial::Uart &uart, SimState &st, uint32_t now_ms,
				 uint32_t hb_timeout_ms, bool corrupt) {
	uint8_t payload[sizeof(mc::proto::StatusPayload)]{};

	const bool hb_timeout =
		st.last_ping_ms > 0 &&
		(uint32_t)(now_ms - st.last_ping_ms) > hb_timeout_ms;

	bool ttl_expired = false;
	if (st.has_drive && st.auto_mode && !st.kill_latched) {
		if (st.target_ttl_ms == 0) {
			ttl_expired = true;
		} else if ((uint32_t)(now_ms - st.last_drive_ms) > st.target_ttl_ms) {
			ttl_expired = true;
		}
	}

	const bool auto_inactive = st.has_drive && !st.auto_mode;
	const bool applied =
		st.auto_mode && !st.kill_latched && st.has_drive && !ttl_expired;

	uint16_t faults = 0;
	if (st.kill_latched)
		faults |= 1u << 0;
	if (hb_timeout)
		faults |= 1u << 1;
	if (ttl_expired)
		faults |= 1u << 2;
	if (auto_inactive)
		faults |= 1u << 3;

	const int16_t speed_now = applied ? st.target_speed_mm_s : 0;
	const int16_t steer_now = applied ? st.target_steer_cdeg : 0;

	uint16_t age_ms = 0xFFFF;
	if (st.has_drive) {
		const uint32_t age = now_ms - st.last_drive_ms;
		age_ms = (age > 0xFFFFu) ? 0xFFFFu : (uint16_t)age;
	}

	payload[0] = applied ? (uint8_t)(st.last_drive_seq & 0xFFu) : 0;
	payload[1] = st.auto_mode ? 1 : 0;
	wr16(payload + 2, faults);
	wr16(payload + 4, (uint16_t)speed_now);
	wr16(payload + 6, (uint16_t)steer_now);
	wr16(payload + 8, age_ms);

	uint8_t frame[mc::proto::MAX_FRAME_ENCODED];
	size_t frame_len = 0;
	if (!mc::proto::PacketWriter::build(frame, sizeof(frame), frame_len,
										mc::proto::Type::STATUS, 0, 0, payload,
										sizeof(payload))) {
		return false;
	}
	if (corrupt && frame_len > 2) {
		frame[1] ^= 0xFF;
	}
	return uart.write(frame, (int)frame_len) > 0;
}

bool send_ack(mc::serial::Uart &uart, uint16_t seq) {
	uint8_t frame[mc::proto::MAX_FRAME_ENCODED];
	size_t frame_len = 0;
	if (!mc::proto::PacketWriter::build(frame, sizeof(frame), frame_len,
										mc::proto::Type::ACK, 0, seq, nullptr,
										0)) {
		return false;
	}
	return uart.write(frame, (int)frame_len) > 0;
}

void handle_frame(const mc::proto::FrameView &f, SimState &st, uint32_t now_ms,
				  mc::serial::Uart &uart) {
	switch (static_cast< mc::proto::Type >(f.type())) {
	case mc::proto::Type::DRIVE: {
		if (f.payload_len != 8)
			break;
		st.target_steer_cdeg = rd16s(f.payload + 0);
		st.target_speed_mm_s = rd16s(f.payload + 2);
		st.target_ttl_ms = rd16u(f.payload + 4);
		st.last_drive_ms = now_ms;
		st.last_drive_seq = f.seq();
		st.has_drive = true;
		break;
	}
	case mc::proto::Type::MODE_SET: {
		if (f.payload_len != 1)
			break;
		st.auto_mode = (f.payload[0] == 1);
		break;
	}
	case mc::proto::Type::KILL:
		st.kill_latched = true;
		break;
	case mc::proto::Type::PING:
		st.last_ping_ms = now_ms;
		break;
	default:
		break;
	}

	if ((f.flags() & mc::proto::FLAG_ACK_REQ) ||
		f.type() == (uint8_t)mc::proto::Type::PING) {
		(void)send_ack(uart, f.seq());
	}
}

Config parse_args(int argc, char **argv) {
	Config cfg;
	cfg.dev = "/dev/ttyUSB0";
	cfg.log_path = sim_esp32d_cfg::DEFAULT_LOG;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--dev" && i + 1 < argc) {
			cfg.dev = argv[++i];
		} else if (a == "--log" && i + 1 < argc) {
			cfg.log_path = argv[++i];
		} else if (a == "--baud" && i + 1 < argc) {
			cfg.baud = std::atoi(argv[++i]);
		} else if (a == "--status-ms" && i + 1 < argc) {
			cfg.status_interval_ms = (uint32_t)std::atoi(argv[++i]);
		} else if (a == "--status-delay-ms" && i + 1 < argc) {
			cfg.status_delay_ms = (uint32_t)std::atoi(argv[++i]);
		} else if (a == "--status-drop-every" && i + 1 < argc) {
			cfg.status_drop_every = (uint32_t)std::atoi(argv[++i]);
		} else if (a == "--status-corrupt-every" && i + 1 < argc) {
			cfg.status_corrupt_every = (uint32_t)std::atoi(argv[++i]);
		} else if (a == "--hb-ms" && i + 1 < argc) {
			cfg.hb_timeout_ms = (uint32_t)std::atoi(argv[++i]);
		}
	}
	if (cfg.status_interval_ms == 0)
		cfg.status_interval_ms = 50;
	return cfg;
}

} // namespace

int main(int argc, char **argv) {
	std::signal(SIGINT, on_sigint);
	std::signal(SIGTERM, on_sigint);

	const Config cfg = parse_args(argc, argv);
	auto &logger = mc::core::Logger::instance();
	if (!cfg.log_path.empty()) {
		mc::core::ensure_dir(mc::core::dir_of(cfg.log_path));
		logger.addSink(std::make_shared< mc::core::FileSink >(cfg.log_path));
	}
	logger.log(mc::core::LogLevel::Info, "sim_esp32d",
			   "start dev=" + cfg.dev + " baud=" + std::to_string(cfg.baud));

	mc::serial::Uart uart;
	if (!uart.open(cfg.dev, cfg.baud)) {
		logger.log(mc::core::LogLevel::Fatal, "sim_esp32d",
				   "failed to open dev " + cfg.dev);
		return 1;
	}

	SimState st;
	mc::proto::PacketReader pr;
	uint32_t next_status_ms = mc::core::Time::ms();

	while (g_run.load()) {
		const uint32_t now_ms = mc::core::Time::ms();
		int timeout_ms = 0;
		if ((int32_t)(next_status_ms - now_ms) > 0) {
			timeout_ms = (int)(next_status_ms - now_ms);
		}

		pollfd pfd{uart.fd(), POLLIN, 0};
		(void)::poll(&pfd, 1, timeout_ms);

		if (pfd.revents & POLLIN) {
			uint8_t buf[512];
			int n = uart.read(buf, (int)sizeof(buf));
			if (n > 0) {
				for (int i = 0; i < n; ++i) {
					if (pr.push(buf[i]) && pr.hasFrame()) {
						handle_frame(pr.frame(), st, now_ms, uart);
						pr.consumeFrame();
					}
				}
			}
		}

		if ((int32_t)(now_ms - next_status_ms) >= 0) {
			st.status_seq++;
			if (cfg.status_delay_ms > 0) {
				usleep(cfg.status_delay_ms * 1000u);
			}
			const bool drop = cfg.status_drop_every > 0 &&
							  (st.status_seq % cfg.status_drop_every) == 0;
			const bool corrupt =
				cfg.status_corrupt_every > 0 &&
				(st.status_seq % cfg.status_corrupt_every) == 0;
			const uint32_t send_ms = mc::core::Time::ms();
			if (!drop) {
				(void)send_status(uart, st, send_ms, cfg.hb_timeout_ms,
								  corrupt);
			}
			next_status_ms = send_ms + cfg.status_interval_ms;
		}
	}

	uart.close();
	logger.log(mc::core::LogLevel::Info, "sim_esp32d", "shutdown");
	logger.shutdown();
	return 0;
}
