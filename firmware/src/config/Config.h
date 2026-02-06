#pragma once

#include <stdint.h>

// Build flag: disable MANUAL (pad) support in production by default.
#ifndef MC_ENABLE_MANUAL
#define MC_ENABLE_MANUAL 0
#endif

#if defined(__has_include)
#if __has_include("mc_config/vehicle_limits.h")
#include "mc_config/vehicle_limits.h"
#endif
#endif

namespace cfg {
// clang-format off

//------------------------------------------------------------------------------
// シリアルログ出力（USB/UART0）
//------------------------------------------------------------------------------
static constexpr uint32_t LOG_BAUD             = 115200;
static constexpr uint32_t LOG_STARTUP_DELAY_MS = 200;

//------------------------------------------------------------------------------
// UART2（RPi ↔ ESP32）
//------------------------------------------------------------------------------

// ピン設定
static constexpr int SERIAL_RX_PIN             = 16;
static constexpr int SERIAL_TX_PIN             = 17;

// 通信設定
static constexpr uint32_t SERIAL_BAUD          = 921600;

//------------------------------------------------------------------------------
// TSD20 (1D LiDAR)
//------------------------------------------------------------------------------

// I2C pins
static constexpr int TSD20_SDA_PIN             = 32;
static constexpr int TSD20_SCL_PIN             = 33;

// I2C settings
static constexpr uint32_t TSD20_I2C_HZ         = 400000;
static constexpr uint8_t TSD20_I2C_ADDR        = 0x52;

// UART "Change IIC" settings
static constexpr uint32_t TSD20_UART_BAUD_PRIMARY  = 460800;
static constexpr uint32_t TSD20_UART_BAUD_FALLBACK = 115200;

// Wiring fallback
static constexpr bool TSD20_ALLOW_PIN_SWAP     = true;

// Read/processing
static constexpr bool TSD20_ENABLE              = true;
static constexpr uint32_t TSD20_READ_INTERVAL_MS = 10;
static constexpr uint32_t TSD20_INIT_RETRY_MS     = 2000;
static constexpr uint8_t TSD20_MAX_FAILS         = 5;
static constexpr bool TSD20_REQUIRE_OK           = true;
static constexpr bool TSD20_CLAMP_IN_MANUAL      = false;
static constexpr bool TSD20_SET_FREQ_ON_BOOT     = true;
static constexpr uint16_t TSD20_TARGET_HZ        = 100;
static constexpr uint16_t TSD20_MARGIN_MM        = 120; // base stop margin
static constexpr uint16_t TSD20_MARGIN_MIN_MM    = 80;  // min for v_cap calc
static constexpr uint16_t TSD20_MARGIN_RELAX_MM  = 40;  // relax at full steer
// センサ読取〜モータ減速完了までの総遅延。40msは不足しがち。
// モータSlewRate: 255/2400≈106ms要するため、100ms以上推奨。
static constexpr uint16_t TSD20_LATENCY_MS       = 100;
static constexpr bool TSD20_PREDICT_ENABLE       = true;
static constexpr uint16_t TSD20_PREDICT_MARGIN_MAX_MM = 350;
static constexpr int TSD20_PREDICT_ACCEL_MAX_MM_S2 = 8000;

// Distance clamp (tune to your course)
static constexpr uint16_t TSD20_STOP_DISTANCE_MM    = 500;
static constexpr uint16_t TSD20_SLOWDOWN_DISTANCE_MM = 1000;

//------------------------------------------------------------------------------
// 自動モードのハートビート監視タイムアウト
//------------------------------------------------------------------------------
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 200;

//------------------------------------------------------------------------------
// Drive用の監視タイムアウトと角度閾値
//------------------------------------------------------------------------------

// ウォッチドッグ
static constexpr uint32_t DRIVE_TIMEOUT_MS     = 250;

// 角度閾値
static constexpr int DRIVE_FRONT_AREA_DEG      = 30;
static constexpr int DRIVE_CURVE_AREA_DEG      = 70;
static constexpr int FULL_CIRCLE_DEG           = 360;

// その他
static constexpr int DRIVE_AVE_DEG_NUM         = 1;

//------------------------------------------------------------------------------
// IMU (MPU-6500)
//------------------------------------------------------------------------------

static constexpr bool IMU_ENABLE                = true;
static constexpr int IMU_SDA_PIN                = 21;
static constexpr int IMU_SCL_PIN                = 22;
static constexpr uint32_t IMU_I2C_HZ            = 400000;
static constexpr uint8_t IMU_I2C_ADDR           = 0x68;
static constexpr uint8_t IMU_WHO_AM_I_0         = 0x70; // MPU-6500
static constexpr uint8_t IMU_WHO_AM_I_1         = 0x68; // MPU-6050互換
static constexpr uint8_t IMU_DLPF_CFG           = 3;    // 44Hz
static constexpr uint8_t IMU_GYRO_FS_SEL        = 3;    // ±2000 dps
static constexpr uint8_t IMU_ACCEL_FS_SEL       = 2;    // ±8g
static constexpr uint32_t IMU_READ_INTERVAL_MS  = 5;    // 200Hz
static constexpr bool IMU_USE_FUSION            = true;
static constexpr float IMU_FUSION_GAIN          = 0.5f;
static constexpr float IMU_FUSION_ACC_REJ_DEG   = 10.0f;
static constexpr float IMU_FUSION_RECOVER_S     = 0.0f;
// Axis map: 0=ax, 1=ay, 2=az (sensor -> vehicle frame)
static constexpr int IMU_AXIS_MAP_X             = 0;
static constexpr int IMU_AXIS_MAP_Y             = 1;
static constexpr int IMU_AXIS_MAP_Z             = 2;
static constexpr int IMU_AXIS_SIGN_X            = 1;
static constexpr int IMU_AXIS_SIGN_Y            = 1;
static constexpr int IMU_AXIS_SIGN_Z            = 1;
static constexpr uint32_t IMU_CALIBRATION_MS    = 1000;
static constexpr uint32_t IMU_CALIB_MIN_SAMPLES = 200;
static constexpr float IMU_CALIB_GYRO_DPS       = 6.0f;
static constexpr int IMU_CALIB_ACCEL_DEV_MM_S2  = 3000;
static constexpr uint32_t IMU_ZUPT_HOLD_MS       = 200;
static constexpr int IMU_ZUPT_A_LONG_MM_S2       = 200;  // 0.2 m/s^2
static constexpr float IMU_ZUPT_GZ_DPS           = 5.0f;
static constexpr int IMU_ZUPT_SPEED_MM_S         = 100;
static constexpr float IMU_V_EST_LEAK_PER_S      = 0.2f;
static constexpr int IMU_V_EST_MAX_MM_S          = mc_config::SPEED_MAX_MM_S;
static constexpr uint32_t IMU_GRAVITY_TAU_MS     = 500;
static constexpr int IMU_ACCEL_NORM_MAX_DEV_MM_S2 = 5000;
static constexpr int IMU_BRAKE_INIT_MM_S2        = 12000;
static constexpr int IMU_BRAKE_MIN_MM_S2         = 8000;
static constexpr int IMU_BRAKE_MAX_MM_S2         = 12000;
static constexpr int IMU_BRAKE_INIT_UNCAL_MM_S2  = 4000;
static constexpr int IMU_BRAKE_MIN_UNCAL_MM_S2   = 3000;
static constexpr int IMU_BRAKE_MAX_UNCAL_MM_S2   = 6000;
static constexpr float IMU_BRAKE_ALPHA_UP        = 0.02f;
static constexpr float IMU_BRAKE_ALPHA_DOWN      = 0.2f;
static constexpr int IMU_BRAKE_DETECT_MM_S2      = 800;
static constexpr int IMU_BRAKE_CMD_DELTA_MM_S    = 200;

//------------------------------------------------------------------------------
// ABS reverse brake
//------------------------------------------------------------------------------

static constexpr bool ABS_ENABLE                = true;
static constexpr bool ABS_ENABLE_IN_MANUAL      = false;
static constexpr uint32_t ABS_PERIOD_MS          = 20;
static constexpr uint8_t ABS_DUTY_MIN            = 20;
static constexpr uint8_t ABS_DUTY_MAX            = 80;
static constexpr float ABS_DUTY_KP               = 0.02f;
static constexpr float ABS_DUTY_KI               = 0.01f;
static constexpr float ABS_A_TARGET_SCALE        = 1.0f;
static constexpr uint16_t ABS_BRAKE_HOLD_MS      = 200;
static constexpr int ABS_REVERSE_MM_S           = 1200;
static constexpr int ABS_SPEED_MARGIN_MM_S      = 150;
static constexpr int ENGINE_RATE_DOWN_BRAKE      = 4000;
static constexpr uint16_t ABS_REVERSE_DISABLE_MARGIN_MM = 150;

//------------------------------------------------------------------------------
// 速度制御（speed_mm_s -> PWM）
//------------------------------------------------------------------------------

static constexpr float SPEED_KP                = 0.02f;
static constexpr float SPEED_KI                = 0.005f;
static constexpr float SPEED_KP_UNCAL          = 0.005f;
static constexpr float SPEED_KI_UNCAL          = 0.0f;
static constexpr int SPEED_I_CLAMP             = 120;
static constexpr int SPEED_DEADBAND_MM_S       = 200;

//------------------------------------------------------------------------------
// ステアサーボ設定（DS3218想定）
//------------------------------------------------------------------------------

// GPIO / PWM
static constexpr int STEER_PIN_SERVO           = 12;
static constexpr int STEER_CHANNEL             = 2;
static constexpr int STEER_PWM_FREQ_HZ         = 50;
static constexpr int STEER_PWM_RESOLUTION_BITS = 16;
static constexpr int STEER_PWM_PERIOD_US       = 1000000 / STEER_PWM_FREQ_HZ;

// パルス幅
static constexpr int STEER_PULSE_MIN_US        = 1200;
static constexpr int STEER_PULSE_MAX_US        = 1750;
static constexpr int STEER_PULSE_CENTER_US     =
	(STEER_PULSE_MIN_US + STEER_PULSE_MAX_US) / 2;

// 角度マッピング（物理可動域に合わせて±25deg）
static constexpr int STEER_ANGLE_CENTER_DEG    = 90;

//------------------------------------------------------------------------------
// モータドライバ（IBT-2）とPWMランプ設定
//------------------------------------------------------------------------------

// GPIO
static constexpr int ENGINE_PIN_RPWM           = 25;
static constexpr int ENGINE_PIN_LPWM           = 26;
static constexpr int ENGINE_PIN_REN            = 27;
static constexpr int ENGINE_PIN_LEN            = 14;

// PWM
static constexpr int ENGINE_CHANNEL_RPWM       = 0;
static constexpr int ENGINE_CHANNEL_LPWM       = 1;
static constexpr int ENGINE_PWM_FREQ_HZ        = 20000;
static constexpr int ENGINE_PWM_RES_BITS       = 8;

// ランプ（SlewRateLimiterは per-second）
static constexpr float ENGINE_RATE_UP          = 1600.0f;
// 減速を速くしてブレーキ応答を改善（255→0 を約64msに短縮）
static constexpr float ENGINE_RATE_DOWN        = 4000.0f;
static constexpr int ENGINE_RAMP_DELAY_MS      = 10;
static constexpr int ENGINE_SPEED_LIMIT        = 255;
static constexpr uint32_t ENGINE_DEADTIME_US   = 800;

//------------------------------------------------------------------------------
// 手動操作の上限（ゲームパッド）
//------------------------------------------------------------------------------

// 速度
static constexpr int   MANUAL_SPEED_MAX  = 180;
static constexpr int   MANUAL_SPEED_STEP = 6;

// 操舵
static constexpr float MANUAL_STEER_DEG  = 25.0f;

//------------------------------------------------------------------------------
// 通信/テレメトリ
//------------------------------------------------------------------------------

// 送信周期
static constexpr uint32_t STATUS_INTERVAL_MS       = 10;

// プロトコルトレース
static constexpr bool PROTO_TRACE_ENABLE           = true;
static constexpr bool PROTO_TRACE_VERBOSE          = false;
static constexpr uint32_t PROTO_TRACE_SUMMARY_MS   = 1000;

// ACKコード
static constexpr uint8_t ACK_CODE_OK               = 0;
static constexpr uint8_t ACK_CODE_VERSION_MISMATCH = 2;
static constexpr uint8_t ACK_CODE_INVALID_PAYLOAD  = 3;
static constexpr uint8_t ACK_CODE_UNHANDLED        = 4;
static constexpr uint8_t ACK_CODE_INVALID_TTL      = 5;

//------------------------------------------------------------------------------
// Autoコマンド
//------------------------------------------------------------------------------

static constexpr uint8_t AUTO_CMD_SEQ_WINDOW = 128;
static constexpr uint16_t AUTO_CMD_AGE_UNKNOWN_MS = 0xFFFF;

// clang-format on
} // namespace cfg
