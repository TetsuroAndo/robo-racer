#pragma once

#include <cstddef>
#include <stdint.h>

namespace cfg {
// clang-format off

//------------------------------------------------------------------------------
// デバイス/通信のデフォルト値
//------------------------------------------------------------------------------

// デバイスパス
static constexpr const char* DEFAULT_LIDAR_DEVICE = "/dev/ttyAMA2";
static constexpr const char* DEFAULT_ESP_DEVICE   = "/dev/ttyAMA0";

// ボーレート
static constexpr int DEFAULT_LIDAR_BAUD = 460800;
static constexpr int DEFAULT_ESP_BAUD   = 921600;

//------------------------------------------------------------------------------
// Sender（ESP32送信）
//------------------------------------------------------------------------------

// タイムアウト/周期
static constexpr uint16_t AUTO_TTL_MS            = 100;
static constexpr uint32_t HEARTBEAT_INTERVAL_MS  = 50;
static constexpr uint32_t STATUS_LOG_INTERVAL_MS = 1000;

// 送信値のスケール/上限
static constexpr int SPEED_LIMIT                 = 255;
static constexpr int STEER_CDEG_SCALE            = 100;

// 受信バッファ
static constexpr size_t UART_READ_BUF_SIZE       = 256;

//------------------------------------------------------------------------------
// LIDAR受信/前処理
//------------------------------------------------------------------------------

// バッファ
static constexpr size_t LIDAR_NODE_MAX           = 8192;

// 角度変換（SDK仕様）
static constexpr float LIDAR_ANGLE_Q14_SCALE_DEG = 90.0f;
static constexpr float LIDAR_ANGLE_Q14_DENOM     = 16384.0f;
static constexpr float LIDAR_ANGLE_WRAP_DEG      = 180.0f;
static constexpr float LIDAR_ANGLE_FULL_DEG      = 360.0f;

// 距離フィルタ
static constexpr unsigned int LIDAR_DIST_MIN_MM  = 5;

//------------------------------------------------------------------------------
// Process（簡易戦略）
//------------------------------------------------------------------------------

// 角度範囲
static constexpr float PROCESS_ANGLE_MIN_DEG     = -70.0f;
static constexpr float PROCESS_ANGLE_MAX_DEG     = 70.0f;

// 出力スケール
static constexpr int PROCESS_SPEED_DIV           = 50;
static constexpr float PROCESS_MIN_ANGLE_SIGN    = -1.0f;

// clang-format on
} // namespace cfg
