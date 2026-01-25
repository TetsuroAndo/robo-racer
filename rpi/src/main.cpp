#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"//TODO:作る
#include "sensor_msgs/msg/laser_scan.hpp"

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	std::cout << "\nSignal " << sig << " received, shutting down..." << std::endl;
	g_stop = 1;
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("robo_racer_bridge");
	auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
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

	// メインスレッドでは評価・送信を実行
	while (!g_stop && rclcpp::ok()) {
		std::vector< LidarData > lidarData;

		// 最新のLiDARデータを取得（スレッドセーフ）
		if (lidarReceiver.getLatestData(lidarData)) {
			// データが利用可能
			const ProcResult procResult = process.proc(lidarData);
			sender.send(procResult.speed, procResult.angle);

			// ★追加 (3/3): ここでROS形式に変換して横流し (Publish) する
			auto scan_msg = sensor_msgs::msg::LaserScan();
			scan_msg.header.stamp = node->now();
			scan_msg.header.frame_id = "laser"; // TFの設定と同じ名前にする

			// LiDARのスペック設定 (RPLiDAR A1などの一般的な設定)
			scan_msg.angle_min = 0.0;
			scan_msg.angle_max = 2.0 * M_PI;
			scan_msg.angle_increment = (2.0 * M_PI) / 360.0; // 1度刻み
			scan_msg.range_min = 0.15;
			scan_msg.range_max = 12.0;

			// データを詰める (360度分、初期値は無限大)
			scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());
			for (const auto &data : lidarData) {
				// data.angle は度数法(deg)なので、インデックス(0-359)として使う
				int idx = static_cast<int>(data.angle);
				// 範囲内チェック
				if (idx >= 0 && idx < 360) {
					// mm(整数) を m(浮動小数) に変換して格納
					scan_msg.ranges[idx] = data.distance / 1000.0f;
				}
			}
			scan_pub->publish(scan_msg);
			rclcpp::spin_some(node);
		} 
		else {
			// データがまだ来ていない場合は少し待機
			usleep(1 * 1000);
		}
	}

	// 正常なシャットダウン
	std::cout << "Main thread: stopping LiDAR receiving thread..." << std::endl;
	lidarReceiver.stopReceivingThread();
	rclcpp::shutdown();
	std::cout << "Main thread: shutdown complete" << std::endl;

	return 0;
}
