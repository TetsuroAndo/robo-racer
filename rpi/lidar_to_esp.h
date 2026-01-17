#ifndef RPI_LIDAR_TO_ESP_H
#define RPI_LIDAR_TO_ESP_H

// LiDAR から取得して ESP に送信するメイン処理
// lidar_dev : LiDAR のシリアルデバイス
// lidar_baud: LiDAR のボーレート
// esp_dev   : ESP 側 UART デバイス
int run_lidar_to_esp(const char* lidar_dev, int lidar_baud, const char* esp_dev);

#endif
