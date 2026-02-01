#include "Process.h"
#include "Sender.h"
#include "ShmLidarReceiver.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	std::cout << "\nSignal " << sig << " received, shutting down..."
			  << std::endl;
	g_stop = 1;
}

int main(int argc, char **argv) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	const char *lidar_dev = (argc >= 2) ? argv[1] : cfg::DEFAULT_LIDAR_DEVICE;
	int lidar_baud = (argc >= 3) ? std::atoi(argv[2]) : cfg::DEFAULT_LIDAR_BAUD;
	const char *seriald_sock =
		(argc >= 4) ? argv[3] : cfg::DEFAULT_SERIALD_SOCK;

	(void)lidar_dev;
	(void)lidar_baud;
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

	// メインスレッドでは評価・送信を実行
	while (!g_stop) {
		std::vector< LidarData > lidarData;

		// 最新のLiDARデータを取得（共有メモリ経由）
		if (lidarReceiver.getLatestData(lidarData)) {
			// データが利用可能
			const ProcResult procResult =
				process.proc(lidarData, lastSteerAngle);
			sender.send(procResult.speed, procResult.angle);

			// 次のループのために今回のステアリング角度を保存
			lastSteerAngle = procResult.angle;
		} else {
			// データがまだ来ていない場合は少し待機
			usleep(1 * 1000);
		}
	}

	// 正常なシャットダウン
	std::cout << "Main thread: shutdown complete" << std::endl;

	return 0;
}
