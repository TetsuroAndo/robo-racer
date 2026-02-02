#include "Process.h"
#include "Sender.h"
#include "LidarReceiver.h"
#include "ShmLidarReceiver.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"//TODO:作る

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
    std::cout << "\nSignal " << sig << " received, shutting down..." << std::endl;
    g_stop = 1;
}

int main(int argc, char **argv) {
    // ROS 2ノードの初期化
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

    // バックグラウンドスレッドで LiDAR 受信開始
    lidarReceiver.startReceivingThread();
    std::cout << "Main thread: LiDAR receiving thread started" << std::endl;

    // 前回のステアリング角度を保持
    float lastSteerAngle = 0.0f;

    // メインスレッドでは評価・送信を実行
    while (!g_stop && rclcpp::ok()) {
        std::vector< LidarData > lidarData;

        // 最新のLiDARデータを取得（スレッドセーフ）
        if (lidarReceiver.getLatestData(lidarData)) {
            // データが利用可能
            const ProcResult procResult = process.proc(lidarData, lastSteerAngle);
            sender.send(procResult.speed, procResult.angle);

            // 次のループのために今回のステアリング角度を保存
            lastSteerAngle = static_cast<float>(procResult.angle);

            // --- ROS 2へのデータ配信処理 ---
            auto scan_msg = sensor_msgs::msg::LaserScan();
            scan_msg.header.stamp = node->now();
            scan_msg.header.frame_id = "laser";

            // ★修正: ROS標準に合わせて -π ～ +π に設定
            scan_msg.angle_min = -M_PI;
            scan_msg.angle_max = M_PI;
            scan_msg.angle_increment = (2.0 * M_PI) / 360.0;
            scan_msg.range_min = 0.15;
            scan_msg.range_max = 12.0;

            // 初期値を無限大(未測定)で埋める
            scan_msg.ranges.assign(360, std::numeric_limits<float>::infinity());

            for (const auto &data : lidarData) {
                // 角度を正規化 (-180 ～ +180度)
                float angle_deg = data.angle;
                while (angle_deg > 180.0f) angle_deg -= 360.0f;
                while (angle_deg <= -180.0f) angle_deg += 360.0f;

                // インデックス計算
                // -180度 -> index 0
                //    0度 -> index 180 (正面)
                // +180度 -> index 360 (範囲外)
                int idx = static_cast<int>(angle_deg + 180.0f);

                // 範囲チェック (0～359)
                if (idx >= 0 && idx < 360) {
                    float dist_m = data.distance / 1000.0f; // mm -> m
                    // 距離の範囲チェックも実施
                    if (dist_m >= scan_msg.range_min && dist_m <= scan_msg.range_max) {
                        scan_msg.ranges[idx] = dist_m;
                    }
                }
            }
            scan_pub->publish(scan_msg);
            rclcpp::spin_some(node);
            // -----------------------------

        } else {
            // データがまだ来ていない場合は少し待機
            usleep(1 * 1000);
        }
    }

    std::cout << "Main thread: stopping LiDAR receiving thread..." << std::endl;
    lidarReceiver.stopReceivingThread();
    
    rclcpp::shutdown();
    std::cout << "Main thread: shutdown complete" << std::endl;

    return 0;
}