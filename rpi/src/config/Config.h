#pragma once

#include <cstddef>
#include <stdint.h>

#if defined(__has_include)
#if __has_include("mc_config/vehicle_limits.h")
#include "mc_config/vehicle_limits.h"
#endif
#endif

namespace cfg {
// clang-format off

//------------------------------------------------------------------------------
// デバイス/通信のデフォルト値
//------------------------------------------------------------------------------

// デバイスパス
static constexpr const char* DEFAULT_LIDAR_DEVICE = "/dev/ttyAMA2";
static constexpr const char* DEFAULT_SERIALD_SOCK = "/tmp/roboracer/seriald.sock";
static constexpr const char* DEFAULT_PROCESS_LOG  = "./logs/process_telemetry.jsonl";
static constexpr const char* DEFAULT_METRICSD_LOG = "./logs/metricsd.log";

// ボーレート
static constexpr int DEFAULT_LIDAR_BAUD = 460800;

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
// speed: RPi内部も mm/s（物理）で統一し、ESP32へそのまま送る
// steer: degree → centi-degree

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
// FTG（最遠ビン + 通過幅チェック）
//------------------------------------------------------------------------------

// 角度範囲（1degビン）
static constexpr int FTG_ANGLE_MIN_DEG         = -90;
static constexpr int FTG_ANGLE_MAX_DEG         = 90;
static constexpr int FTG_BIN_COUNT             =
	FTG_ANGLE_MAX_DEG - FTG_ANGLE_MIN_DEG + 1;

// 平滑化（Moving Average）
static constexpr int FTG_SMOOTH_RADIUS_BINS    = 2;   // 3〜5binの中央=2

// 車幅/マージン（メートル）
static constexpr float FTG_CAR_WIDTH_M         = 0.20f; // 20cm
static constexpr float FTG_MARGIN_M            = 0.03f; // 必要なら調整
static constexpr float FTG_CORRIDOR_LOOKAHEAD_M = 0.60f;

// 予測マージン（IMUの速度/加速度で安全側へ補正）
static constexpr bool FTG_PREDICT_ENABLE       = true;
static constexpr uint16_t FTG_PREDICT_LATENCY_MS = 80;
static constexpr int FTG_PREDICT_MARGIN_MAX_MM = 500;
static constexpr int FTG_PREDICT_BRAKE_MM_S2   = 6000;
static constexpr int FTG_PREDICT_BRAKE_MIN_MM_S2 = 1500;
static constexpr int FTG_PREDICT_BRAKE_MAX_MM_S2 = 12000;
static constexpr int FTG_PREDICT_ACCEL_MAX_MM_S2 = 8000;

// 障害物判定
static constexpr int FTG_NEAR_OBSTACLE_MM      = 100;  // 10cm以内でブロック
static constexpr int FTG_WARN_OBSTACLE_MM      = 200;  // テレメトリ警告用

// 速度（距離・ステア連動）
// NOTE: RPi側plannerの速度は mm/s（物理）で統一する。
// 旧: 段階値 speed_input (14..255) を使っていたが、Sender側の換算を廃止する。
static constexpr int FTG_SPEED_MIN_MM_S        =
	(mc_config::SPEED_MAX_MM_S * 14) / mc_config::SPEED_INPUT_LIMIT;
static constexpr int FTG_SPEED_MAX_MM_S        = mc_config::SPEED_MAX_MM_S;
// v_dist の指数飽和パラメータ（m）
static constexpr float FTG_SPEED_R_SAFE_M      = 0.30f; // 30cm以下は最小速度
static constexpr float FTG_SPEED_R_MAX_M       = 1.00f; // 100cm以上は最大速度
static constexpr float FTG_SPEED_K_M           = 0.10f; // 立ち上がり（最速）
//数字が大きいほど加速度は下がる

// コスト関数（目的関数）
static constexpr int FTG_COST_SAFE_MM            = 500;   // ここから回避を開始
static constexpr int FTG_JERK_RELAX_MM           = 300;   // 近距離でジャーク抑制を緩める
static constexpr float FTG_COST_W_OBS            = 8.0f;
static constexpr float FTG_COST_W_TURN           = 0.05f;
static constexpr float FTG_COST_W_DELTA          = 0.3f;
static constexpr float FTG_COST_BETA             = 4.0f;  // soft-argminの鋭さ
static constexpr float FTG_STEER_SLEW_DEG_PER_S  = 120.0f;
static constexpr int FTG_SPEED_WARN_CAP_MM_S     =
	(mc_config::SPEED_MAX_MM_S * 39) / mc_config::SPEED_INPUT_LIMIT;
static constexpr uint16_t FTG_IMU_MAX_AGE_MS     = 200;
static constexpr float FTG_YAW_BIAS_DEG          = 0.0f;
static constexpr float FTG_YAW_BIAS_REF_DPS      = 90.0f;

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
