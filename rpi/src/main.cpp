#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
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

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	Process process;
	Sender sender(seriald_sock);

	// バックグラウンドスレッドで LiDAR 受信開始
	lidarReceiver.startReceivingThread();
	std::cout << "Main thread: LiDAR receiving thread started" << std::endl;

	// 前回のステアリング角度を保持
	float lastSteerAngle = 0.0f;

	// メインスレッドでは評価・送信を実行
	while (!g_stop) {
		std::vector< LidarData > lidarData;

		// 最新のLiDARデータを取得（スレッドセーフ）
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
	std::cout << "Main thread: stopping LiDAR receiving thread..." << std::endl;
	lidarReceiver.stopReceivingThread();
	std::cout << "Main thread: shutdown complete" << std::endl;

	return 0;
}
