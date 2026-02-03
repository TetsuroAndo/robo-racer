#include "Process.h"
#include "Sender.h"
#include "ShmLidarReceiver.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include "mc/core/Log.hpp"
#include "mc/core/Time.hpp"
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	std::cout << "\nSignal " << sig << " received, shutting down..."
			  << std::endl;
	g_stop = 1;
}

static void show_cursor() { std::cout << "\x1b[?25h" << std::flush; }

static std::string make_run_id() {
	std::ostringstream oss;
	oss << std::hex << mc::core::Time::us() << "-p" << (unsigned)getpid();
	return oss.str();
}

static void ensure_dir_for(const std::string &path) {
	if (path.empty())
		return;
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return;
	const std::string dir = path.substr(0, pos);
	const int rc = mkdir(dir.c_str(), 0755);
	if (rc == 0 || errno == EEXIST)
		return;
	std::cerr << "WARN: mkdir failed for log dir: " << dir << std::endl;
}

int main(int argc, char **argv) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);
	std::atexit(show_cursor);

	std::string log_path = cfg::DEFAULT_PROCESS_LOG;
	std::string run_id;
	std::vector< std::string > positional;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--log" && i + 1 < argc) {
			log_path = argv[++i];
		} else if (a == "--run-id" && i + 1 < argc) {
			run_id = argv[++i];
		} else {
			positional.push_back(a);
		}
	}

	const char *lidar_dev = (positional.size() >= 1)
								? positional[0].c_str()
								: cfg::DEFAULT_LIDAR_DEVICE;
	int lidar_baud = (positional.size() >= 2) ? std::atoi(positional[1].c_str())
											  : cfg::DEFAULT_LIDAR_BAUD;
	const char *seriald_sock = (positional.size() >= 3)
								   ? positional[2].c_str()
								   : cfg::DEFAULT_SERIALD_SOCK;

	(void)lidar_dev;
	(void)lidar_baud;
	if (run_id.empty())
		run_id = make_run_id();

	auto &logger = mc::core::Logger::instance();
	logger.setConsoleEnabled(false);
	if (!log_path.empty()) {
		ensure_dir_for(log_path);
		logger.addSink(std::make_shared< mc::core::FileSink >(log_path));
	}

	ShmLidarReceiver lidarReceiver;
	Process process;
	Sender sender(seriald_sock);

	while (!g_stop && !lidarReceiver.connect()) {
		std::cerr << "Waiting for lidar_received shared memory..." << std::endl;
		usleep(200 * 1000);
	}
	if (g_stop)
		return 0;

	// 前回のステアリング角度を保持
	float lastSteerAngle = 0.0f;
	uint64_t tick = 0;

	// メインスレッドでは評価・送信を実行
	while (!g_stop) {
		std::vector< LidarData > lidarData;

		// 最新のLiDARデータを取得（共有メモリ経由）
		if (lidarReceiver.getLatestData(lidarData)) {
			// データが利用可能
			const ProcResult procResult =
				process.proc(lidarData, lastSteerAngle, tick, tick, run_id);
			sender.send(procResult.speed, procResult.angle);

			// 次のループのために今回のステアリング角度を保存
			lastSteerAngle = procResult.angle;
			++tick;
		} else {
			// データがまだ来ていない場合は少し待機
			usleep(1 * 1000);
		}
	}

	// 正常なシャットダウン
	std::cout << "Main thread: shutdown complete" << std::endl;
	logger.shutdown();

	return 0;
}
