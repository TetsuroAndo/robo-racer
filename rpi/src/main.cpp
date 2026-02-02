#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"//TODO:作る

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robo_racer_node");
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

	lidarReceiver.startReceivingThread();
    std::cout << "Main thread: LiDAR receiving thread started" << std::endl;

	while (!g_stop && rclcpp::ok()) {
        std::vector< LidarData > lidarData;

        if (lidarReceiver.getLatestData(lidarData)) {
            // 1. 既存の走行ロジック (Follow the Gap)
            const ProcResult procResult = process.proc(lidarData);
            sender.send(procResult.speed, procResult.angle);

            // 2. ★追加: マッピング用にデータをROSへ横流しする
            auto scan_msg = sensor_msgs::msg::LaserScan();
            scan_msg.header.stamp = node->now();
            scan_msg.header.frame_id = "laser"; // TFの設定と合わせる

            // RPLiDAR A1などの一般的な設定
            scan_msg.angle_min = 0.0;
            scan_msg.angle_max = 2.0 * M_PI;
            scan_msg.angle_increment = (2.0 * M_PI) / 360.0; // 1度刻みと仮定
            scan_msg.range_min = 0.15;
            scan_msg.range_max = 12.0;
            scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());

            // LidarData (mm, degree) -> LaserScan (m, index) 変換
            for (const auto &data : lidarData) {
                // LidarReceiver.h によると angle は度数法 [cite]
                int idx = static_cast<int>(data.angle); 
                // 360度以内に収める処理 (必要に応じて調整)
                if (idx >= 0 && idx < 360) {
                     // mm -> m 変換
                    scan_msg.ranges[idx] = data.distance / 1000.0f;
                }
            }
            scan_pub->publish(scan_msg);
            
            // ROSのコールバック処理（あれば）
            rclcpp::spin_some(node);

        } else {
            usleep(1 * 1000);
        }
    }
    lidarReceiver.stopReceivingThread();
    rclcpp::shutdown();
    std::cout << "Main thread: shutdown complete" << std::endl;

    return 0;
}
