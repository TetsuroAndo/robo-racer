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
#include <algorithm>
#include <limits>

// ROS 2 Header
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
    std::cout << "\nSignal " << sig << " received, shutting down..." << std::endl;
    g_stop = 1;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robo_racer_node");
    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    const char *lidar_dev = (argc >= 2) ? argv[1] : cfg::DEFAULT_LIDAR_DEVICE;
    int lidar_baud = (argc >= 3) ? std::atoi(argv[2]) : cfg::DEFAULT_LIDAR_BAUD;
    const char *seriald_sock = (argc >= 4) ? argv[3] : cfg::DEFAULT_SERIALD_SOCK;

    LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
    Process process;
    Sender sender(seriald_sock);

    lidarReceiver.startReceivingThread();
    std::cout << "Main thread: LiDAR receiving thread started" << std::endl;

    float lastSteerAngle = 0.0f;

    while (!g_stop && rclcpp::ok()) {
        std::vector< LidarData > lidarData;

        if (lidarReceiver.getLatestData(lidarData)) {
            // 1. 走行ロジック
            const ProcResult procResult = process.proc(lidarData, lastSteerAngle);
            sender.send(procResult.speed, procResult.angle);
            lastSteerAngle = static_cast<float>(procResult.angle);

            // 2. マッピング用データ変換 (ROS 2)
            auto scan_msg = sensor_msgs::msg::LaserScan();
            scan_msg.header.stamp = node->now();
            scan_msg.header.frame_id = "laser";

            // 設定: -π ～ +π, 1度刻み
            scan_msg.angle_min = -M_PI;
            scan_msg.angle_max = M_PI;
            scan_msg.angle_increment = (2.0 * M_PI) / 360.0;
            scan_msg.range_min = 0.15;
            scan_msg.range_max = 12.0;

            // サイズを動的に計算 (マジックナンバー 360 の廃止)
            // +0.5f は四捨五入のための補正
            size_t num_readings = static_cast<size_t>((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment + 0.5f);
            scan_msg.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

            for (const auto &data : lidarData) {
                // 角度を正規化 (-π <= angle < +π)
                float angle_rad = data.angle * (M_PI / 180.0f);
                while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
                while (angle_rad <= -M_PI) angle_rad += 2.0 * M_PI;

                // インデックス計算: (angle - min) / inc
                int idx = static_cast<int>((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment);

                // 安全な範囲チェック
                if (idx >= 0 && idx < static_cast<int>(scan_msg.ranges.size())) {
                    float dist_m = data.distance / 1000.0f;
                    if (dist_m >= scan_msg.range_min && dist_m <= scan_msg.range_max) {
                        scan_msg.ranges[idx] = dist_m;
                    }
                }
            }
            scan_pub->publish(scan_msg);
            rclcpp::spin_some(node);

        } else {
            usleep(1 * 1000);
        }
    }

    std::cout << "Main thread: stopping LiDAR receiving thread..." << std::endl;
    lidarReceiver.stopReceivingThread();
    
    rclcpp::shutdown();
    std::cout << "Main thread: shutdown complete" << std::endl;

    return 0;
}