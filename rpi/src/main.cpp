#include "MotionState.h"
#include "Process.h"
#include "Sender.h"
#include "ShmLidarReceiver.h"
#include "config/Config.h"
#include "mc/core/Log.hpp"
#include "mc/core/Path.hpp"
#include "mc/core/Signal.hpp"
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
static volatile sig_atomic_t g_last_sig = 0;

static void on_sig(int sig) {
	g_last_sig = sig;
	g_stop = 1;
}

static std::string make_run_id() {
	std::ostringstream oss;
	oss << std::hex << mc::core::Time::us() << "-p" << (unsigned)getpid();
	return oss.str();
}

int main(int argc, char **argv) {
	mc::core::setup_signal_handlers(&g_stop, on_sig);
	mc::core::register_atexit_show_cursor();

	std::string log_path = cfg::DEFAULT_PROCESS_LOG;
	std::string metrics_log_path = cfg::DEFAULT_METRICSD_LOG;
	std::string run_id;
	double telemetry_hz = cfg::TELEMETRY_DEFAULT_HZ;
	TelemetryLevel telemetry_level = TelemetryLevel::Basic;
	std::vector< std::string > positional;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--log" && i + 1 < argc) {
			log_path = argv[++i];
		} else if (a == "--metrics-log" && i + 1 < argc) {
			metrics_log_path = argv[++i];
		} else if (a == "--telemetry-hz" && i + 1 < argc) {
			telemetry_hz = std::atof(argv[++i]);
		} else if (a == "--telemetry-level" && i + 1 < argc) {
			std::string lv = argv[++i];
			if (lv == "full")
				telemetry_level = TelemetryLevel::Full;
			else
				telemetry_level = TelemetryLevel::Basic;
		} else if (a == "--run-id" && i + 1 < argc) {
			run_id = argv[++i];
		} else {
			positional.push_back(a);
		}
	}
	if (!(telemetry_hz > 0.0)) {
		std::cerr << "Invalid --telemetry-hz; using default "
				  << cfg::TELEMETRY_DEFAULT_HZ << "\n";
		telemetry_hz = cfg::TELEMETRY_DEFAULT_HZ;
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
		mc::core::ensure_dir_for(log_path);
		logger.addSink(std::make_shared< mc::core::FileSink >(log_path));
	}

	ShmLidarReceiver lidarReceiver;
	TelemetryEmitter telemetry;
	telemetry.setMetricsLogPath(metrics_log_path);
	telemetry.setRateHz(telemetry_hz);
	telemetry.setLevel(telemetry_level);
	Process process(&telemetry);
	Sender sender(seriald_sock, &telemetry);

	uint64_t last_wait_log_us = 0;
	while (!g_stop && !lidarReceiver.connect()) {
		const uint64_t now_us = mc::core::Time::us();
		if (now_us - last_wait_log_us > 1000 * 1000) {
			MC_LOGW("main", "Waiting for lidar_received shared memory...");
			last_wait_log_us = now_us;
		}
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
			const uint64_t scan_id = lidarReceiver.lastSeq();
			sender.poll();
			MotionState motion{};
			const MotionState *motion_ptr =
				sender.motion(motion) ? &motion : nullptr;
			// データが利用可能
			const ProcResult procResult = process.proc(
				lidarData, lastSteerAngle, tick, scan_id, run_id, motion_ptr);
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
	if (g_last_sig != 0) {
		MC_LOGI("main", "Signal " + std::to_string((int)g_last_sig) +
							" received, shutting down...");
	}
	MC_LOGI("main", "shutdown complete");
	telemetry.shutdownUi();
	logger.shutdown();

	return 0;
}
