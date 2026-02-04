#include "config/Config.h"
#include <mc/core/Log.hpp>
#include <mc/core/Path.hpp>
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>

#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

static void usage() {
	std::cerr
		<< "serialctl --uds /tmp/roboracer/seriald.sock [--log path] "
		   "[drive|kill|mode] ...\n"
		   "  drive --seq N --steer_cdeg X --speed_mm_s V --ttl_ms T --dist_mm "
		   "D\n"
		   "  kill  --seq N\n"
		   "  mode  --seq N --mode 0|1\n";
}

static int argi(int &i, int argc, char **argv) {
	if (i + 1 >= argc)
		return 0;
	return std::stoi(argv[++i]);
}

static bool check_tmp_socket_dir_safe(const std::string &sock_path) {
	if (sock_path.rfind("/tmp/", 0) != 0)
		return true;
	const std::string dir = mc::core::dir_of(sock_path);
	if (dir.empty())
		return true;
	struct stat st{};
	if (::stat(dir.c_str(), &st) != 0) {
		std::cerr << "serialctl: socket dir stat failed: " << dir << "\n";
		return false;
	}
	if (!S_ISDIR(st.st_mode)) {
		std::cerr << "serialctl: socket dir is not a directory: " << dir
				  << "\n";
		return false;
	}
	const uid_t uid = ::getuid();
	if (st.st_uid != 0 && st.st_uid != uid) {
		std::cerr << "serialctl: socket dir owner mismatch: " << dir << "\n";
		return false;
	}
	if (st.st_mode & (S_IWGRP | S_IWOTH)) {
		std::cerr << "serialctl: socket dir is group/world-writable: " << dir
				  << "\n";
		return false;
	}
	return true;
}

int main(int argc, char **argv) {
	using mc::core::Logger;
	Logger::instance().setLevel(mc::core::LogLevel::Info);

	std::string uds = "/tmp/roboracer/seriald.sock";
	std::string log_path = serialctl_cfg::DEFAULT_LOG;
	std::string cmd;

	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--uds" && i + 1 < argc)
			uds = argv[++i];
		else if (a == "--log" && i + 1 < argc)
			log_path = argv[++i];
		else if (a == "drive" || a == "kill" || a == "mode")
			cmd = a;
	}
	if (cmd.empty()) {
		usage();
		return 1;
	}

	if (!log_path.empty()) {
		mc::core::ensure_dir(mc::core::dir_of(log_path));
		Logger::instance().addSink(
			std::make_shared< mc::core::FileSink >(log_path));
	}

	uint16_t seq = 1;
	int steer_cdeg = 0;
	int speed_mm_s = 0;
	int ttl_ms = 100;
	int dist_mm = 0;
	int mode = 1;

	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--seq")
			seq = (uint16_t)argi(i, argc, argv);
		else if (a == "--steer_cdeg")
			steer_cdeg = argi(i, argc, argv);
		else if (a == "--speed_mm_s")
			speed_mm_s = argi(i, argc, argv);
		else if (a == "--ttl_ms")
			ttl_ms = argi(i, argc, argv);
		else if (a == "--dist_mm")
			dist_mm = argi(i, argc, argv);
		else if (a == "--mode")
			mode = argi(i, argc, argv);
	}

	if (!check_tmp_socket_dir_safe(uds))
		return 1;

	mc::ipc::UdsClient c;
	if (!c.connect(uds)) {
		MC_LOGF("serialctl", "connect failed: " + uds);
		return 1;
	}

	uint8_t out[mc::proto::MAX_FRAME_ENCODED];
	size_t out_len = 0;

	if (cmd == "kill") {
		uint8_t payload[2] = {0, 0};
		if (!mc::proto::PacketWriter::build(out, sizeof(out), out_len,
											mc::proto::Type::KILL, 0, seq,
											payload, sizeof(payload))) {
			MC_LOGF("serialctl", "build kill failed");
			return 1;
		}
	} else if (cmd == "mode") {
		uint8_t payload[1] = {(uint8_t)mode};
		if (!mc::proto::PacketWriter::build(out, sizeof(out), out_len,
											mc::proto::Type::MODE_SET, 0, seq,
											payload, sizeof(payload))) {
			MC_LOGF("serialctl", "build mode failed");
			return 1;
		}
	} else if (cmd == "drive") {
		uint8_t payload[8]{};
		auto put_i16 = [&](int off, int v) {
			int16_t x = (int16_t)v;
			payload[off + 0] = (uint8_t)(x & 0xFF);
			payload[off + 1] = (uint8_t)((x >> 8) & 0xFF);
		};
		auto put_u16 = [&](int off, int v) {
			uint16_t x = (uint16_t)v;
			payload[off + 0] = (uint8_t)(x & 0xFF);
			payload[off + 1] = (uint8_t)((x >> 8) & 0xFF);
		};
		put_i16(0, steer_cdeg);
		put_i16(2, speed_mm_s);
		put_u16(4, ttl_ms);
		put_u16(6, dist_mm);

		if (!mc::proto::PacketWriter::build(out, sizeof(out), out_len,
											mc::proto::Type::DRIVE, 0, seq,
											payload, sizeof(payload))) {
			MC_LOGF("serialctl", "build drive failed");
			return 1;
		}
	}

	ssize_t w = ::send(c.fd(), out, out_len, MSG_NOSIGNAL);
	if (w < 0) {
		MC_LOGF("serialctl", "send failed");
		return 1;
	}
	MC_LOGI("serialctl", "sent bytes=" + std::to_string((int)w));
	Logger::instance().shutdown();
	return 0;
}
