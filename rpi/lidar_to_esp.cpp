// lidar_to_esp.cpp

#include "lidar_to_esp.h"
#include "uart.h"

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>

// 小数点切り捨て / unsigned int
static inline unsigned int deg_floor_u(const sl_lidar_response_measurement_node_hq_t &n) {
    return (static_cast<unsigned int>(n.angle_z_q14) * 90U) >> 14;
}

static inline unsigned int dist_mm_floor_u(const sl_lidar_response_measurement_node_hq_t &n) {
    return static_cast<unsigned int>(n.dist_mm_q2 >> 2);
}

int run_lidar_to_esp(const char* lidar_dev_c, int lidar_baud, const char* esp_dev) {

    // ESP送信用UART
    int esp_fd = uart_open_writeonly(esp_dev, 115200);
    if (esp_fd < 0) {
        std::fprintf(stderr, "ESP open failed (%s): %s\n",
                     esp_dev, std::strerror(errno));
        return 1;
    }

    // --- Channel 作成 ---
    auto channelRes = sl::createSerialPortChannel(
        std::string(lidar_dev_c), lidar_baud);

    if (!channelRes) {
        std::fprintf(stderr, "Failed to create serial channel (%s, %d)\n",
                     lidar_dev_c, lidar_baud);
        close(esp_fd);
        return 1;
    }
    sl::IChannel* channel = channelRes.value;

    // --- Driver 作成 ---
    auto lidarRes = sl::createLidarDriver();
    if (!lidarRes) {
        std::fprintf(stderr, "Failed to create lidar driver\n");
        delete channel;
        close(esp_fd);
        return 1;
    }
    sl::ILidarDriver* lidar = lidarRes.value;

    // connect
    sl_result res = lidar->connect(channel);
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "Failed to connect to LIDAR: %08x\n",
                     (unsigned int)res);
        delete lidar;
        delete channel;
        close(esp_fd);
        return 1;
    }

    // デバイス情報/ヘルス確認（SDKサンプルと同じ順序）
    sl_lidar_response_device_info_t devinfo;
    res = lidar->getDeviceInfo(devinfo);
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "getDeviceInfo failed: %08x\n",
                     (unsigned int)res);
        lidar->disconnect();
        delete lidar;
        delete channel;
        close(esp_fd);
        return 1;
    }

    sl_lidar_response_device_health_t health;
    res = lidar->getHealth(health);
    if (!SL_IS_OK(res) || health.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "getHealth failed or unhealthy: %08x status=%u\n",
                     (unsigned int)res, (unsigned int)health.status);
        if (SL_IS_OK(res)) lidar->reset();
        lidar->disconnect();
        delete lidar;
        delete channel;
        close(esp_fd);
        return 1;
    }

    // モータ回転（サンプルと同じく開始前に回す）
    res = lidar->setMotorSpeed();
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "setMotorSpeed failed: %08x\n",
                     (unsigned int)res);
        lidar->disconnect();
        delete lidar;
        delete channel;
        close(esp_fd);
        return 1;
    }

    // スキャン開始
    sl::LidarScanMode scanMode;
    res = lidar->startScan(false, true, 0, &scanMode);
	std::cerr << "LIDAR startScan result: " << res
	          << ", mode=" << scanMode.scan_mode << std::endl;
    if (!SL_IS_OK(res)) {
        std::fprintf(stderr, "startScan failed: %08x\n",
                     (unsigned int)res);
        lidar->stop();
        lidar->setMotorSpeed(0);
        lidar->disconnect();
        delete lidar;
        delete channel;
        close(esp_fd);
        return 1;
    }

    std::array<unsigned int, 360> dist_mm;
    sl_lidar_response_measurement_node_hq_t nodes[8192];

    while (true) {
        dist_mm.fill(0);

        size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);
        res = lidar->grabScanDataHq(nodes, nodeCount);
        if (SL_IS_FAIL(res)) {
            usleep(1000);
            continue;
        }

        lidar->ascendScanData(nodes, nodeCount);

        for (size_t i = 0; i < nodeCount; i++) {
            unsigned int deg = deg_floor_u(nodes[i]);
            if (deg >= 360) continue;
            dist_mm[deg] = dist_mm_floor_u(nodes[i]);
        }

        for (unsigned int deg = 0; deg < 360; deg++) {
            char line[32];
            int n = std::snprintf(line, sizeof(line),
                                  "%u,%u\n", dist_mm[deg], deg);
            if (n > 0) write(esp_fd, line, (size_t)n);
        }
    }

    // 実際にはここには来ない（停止制御は main 側でやる）
    lidar->stop();
    lidar->setMotorSpeed(0);
    lidar->disconnect();

    delete lidar;
    delete channel;
    close(esp_fd);
    return 0;
}
