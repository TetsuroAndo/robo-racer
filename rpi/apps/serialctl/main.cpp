#include "config/Config.h"
#include <mc/core/Log.hpp>
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <vector>

static void ensure_dir_(const std::string &path) {
	if (path.empty())
		return;

	// 親ディレクトリを再帰的に作成
	const size_t pos = path.find_last_of('/');
	if (pos != std::string::npos) {
		if (pos == 0) {
			// 絶対パス (例: /tmp) の場合、ルートディレクトリは既に存在するのでスキップ
			// 何もせず次のmkdirステップに進む
		} else {
			const std::string parent = path.substr(0, pos);
			ensure_dir_(parent);
		}
	}

	const int rc = mkdir(path.c_str(), 0755);
	if (rc == 0 || errno == EEXIST)
		return;

	// mkdir が EEXIST 以外で失敗した場合は、原因を標準エラー出力に出力する
	const int err = errno;
	std::cerr << "Failed to create directory '" << path
		  << "': " << std::strerror(err)
		  << " (errno=" << err << ")\n";
}

static std::string dir_of_(const std::string &path) {
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return std::string();
	return path.substr(0, pos);
}

static void usage() {
	std::cerr
		<< "serialctl --uds /run/roboracer/seriald.sock [--log path] "
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

int main(int argc, char **argv) {
	using mc::core::Logger;
	Logger::instance().setLevel(mc::core::LogLevel::Info);

	std::string uds = "/run/roboracer/seriald.sock";
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
		ensure_dir_(dir_of_(log_path));
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
