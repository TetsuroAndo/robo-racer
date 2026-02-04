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
static constexpr const char* DEFAULT_SERIALD_SOCK = "/tmp/roboracer/seriald.sock";
static constexpr const char* DEFAULT_PROCESS_LOG  = "./logs/process_telemetry.jsonl";
static constexpr const char* DEFAULT_METRICSD_LOG = "./logs/metricsd.log";

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
static constexpr uint32_t ACK_TIMEOUT_MS         = 50;
static constexpr uint8_t ACK_MAX_RETRY           = 3;
static constexpr uint32_t STATUS_DEAD_MS         = 300;

// 送信値のスケール/上限（ESP32側の単位に合わせる）
// speed: RPi内部は旧PWM相当(-255..255) → ESP32はmm/s(±2000想定)
static constexpr int SPEED_INPUT_LIMIT           = 255;
static constexpr int SPEED_MM_S_MAX              = 2000;
// steer: degree → centi-degree
static constexpr int STEER_CDEG_SCALE            = 100;
static constexpr int STEER_CDEG_MAX              = 3000;

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

// 角度範囲（操舵判定と減速判定を分離）
static constexpr float PROCESS_HANDLE_ANGLE_MIN_DEG = -90.0f; // ハンドリング評価
static constexpr float PROCESS_HANDLE_ANGLE_MAX_DEG = 90.0f;
static constexpr float PROCESS_SLOW_ANGLE_MIN_DEG   = -70.0f; // 減速評価
static constexpr float PROCESS_SLOW_ANGLE_MAX_DEG   = 70.0f;

// 出力スケール
static constexpr int PROCESS_SPEED_DIV           = 50;
static constexpr int PROCESS_MAX_SPEED           = 100;  // Raspberry Pi内部での最大速度
// FTGでbest角(B)を操舵基準に統一したため符号を一致させる
static constexpr float PROCESS_MIN_ANGLE_SIGN    = 1.0f;

// FTG基準に合わせて角度補正を外す（1.0 = 補正なし）
// ステアリングゲイン（浅めの角度を補正: 1.0 = 補正なし、> 1.0 で角度を強める）
static constexpr float PROCESS_STEER_GAIN        = 1.0f;

// 障害物距離による速度制限
static constexpr int PROCESS_MIN_DIST_SAFE_MM    = 400;   // 安全距離: これ以上なら通常速度
static constexpr int PROCESS_MIN_DIST_STOP_MM    = 150;   // 停止距離: これ以下なら停止
static constexpr float PROCESS_MIN_DIST_SPEED_FACTOR = 0.5f; // 減速係数: 0.5 = 50% 速度

// 急カーブによる速度制限
static constexpr int STEER_ANGLE_MAX_DEG         = 30;    // サーボの物理的上限
static constexpr float STEER_CURVE_SPEED_FACTOR  = 0.7f;  // 曲率係数: 計算角度/上限 の比率で速度を調整

// 前回ステアリング角度を考慮した障害物評価
static constexpr float PROCESS_STEER_WINDOW_HALF_DEG = 25.0f;  // 前回ステアリング角度の±25度を評価範囲とする

// ステアリング方向への優先度: 現在の方向に近い角度を優先（0.0=優先度なし、1.0=最大優先度）
static constexpr float PROCESS_DIRECTION_WEIGHT = 0.5f;  // 現在方向との角度差を考慮する程度

//------------------------------------------------------------------------------
// Telemetry（観測/可視化）
//------------------------------------------------------------------------------

static constexpr double TELEMETRY_DEFAULT_HZ = 10.0;
static constexpr float TELEMETRY_CANDIDATE_EVENT_DEG = 10.0f;
static constexpr uint16_t TELEMETRY_CPU_WARN_PERMILLE = 800;
static constexpr uint16_t TELEMETRY_CPU_CRIT_PERMILLE = 950;
static constexpr uint16_t TELEMETRY_TEMP_WARN_CDEG = 7000;
static constexpr uint16_t TELEMETRY_TEMP_CRIT_CDEG = 8000;
static constexpr int TELEMETRY_DIST_BAR_MAX_MM = 12000;
static constexpr size_t TELEMETRY_BAR_WIDTH = 12;
static constexpr size_t TELEMETRY_SPARK_LEN = 20;
static constexpr size_t TELEMETRY_COMPASS_ROWS = 6;
static constexpr size_t TELEMETRY_COMPASS_MIN_WIDTH = 21;
static constexpr int TELEMETRY_NEAR_EMPH_MM = 1000;
static constexpr uint32_t TELEMETRY_SCAN_AGE_WARN_MS = 100;
static constexpr uint32_t TELEMETRY_SCAN_AGE_CRIT_MS = 200;
static constexpr float TELEMETRY_BEST_JUMP_DEG = 15.0f;
static constexpr float TELEMETRY_SLOWDOWN_SF = 0.5f;
static constexpr uint32_t TELEMETRY_LATENCY_WARN_MS = 20;
static constexpr uint32_t TELEMETRY_LATENCY_CRIT_MS = 50;
static constexpr float TELEMETRY_TTL_WARN_FACTOR = 1.0f;
static constexpr float TELEMETRY_TTL_CRIT_FACTOR = 2.0f;

// clang-format on
} // namespace cfg
