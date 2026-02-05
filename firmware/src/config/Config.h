#pragma once

#include <stdint.h>

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
static constexpr uint32_t TSD20_I2C_HZ         = 100000;
static constexpr uint8_t TSD20_I2C_ADDR        = 0x52;

// UART "Change IIC" settings
static constexpr uint32_t TSD20_UART_BAUD_PRIMARY  = 460800;
static constexpr uint32_t TSD20_UART_BAUD_FALLBACK = 115200;

// Wiring fallback
static constexpr bool TSD20_ALLOW_PIN_SWAP     = true;

// Read/processing
static constexpr bool TSD20_ENABLE              = true;
static constexpr uint32_t TSD20_READ_INTERVAL_MS = 50;
static constexpr uint32_t TSD20_INIT_RETRY_MS     = 2000;
static constexpr uint8_t TSD20_MAX_FAILS         = 5;
static constexpr bool TSD20_REQUIRE_OK           = true;
static constexpr bool TSD20_CLAMP_IN_MANUAL      = false;

// Distance clamp (tune to your course)
static constexpr uint16_t TSD20_STOP_DISTANCE_MM    = 400;
static constexpr uint16_t TSD20_SLOWDOWN_DISTANCE_MM = 800;

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
static constexpr int DRIVE_SPEED_MAX           = 255;
static constexpr int DRIVE_AVE_DEG_NUM         = 1;
static constexpr int DRIVE_SPEED_MAX_MM_S      = 13889; // 50km/h

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
static constexpr float STEER_ANGLE_MIN_DEG     = -25.0f;
static constexpr float STEER_ANGLE_MAX_DEG     = 25.0f;
static constexpr int STEER_ANGLE_MIN_CDEG      = -2500;
static constexpr int STEER_ANGLE_MAX_CDEG      = 2500;
static constexpr int STEER_ANGLE_CENTER_DEG    = 90;
static constexpr float STEER_CDEG_SCALE        = 100.0f;

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

// ランプ
static constexpr int ENGINE_SPEED_STEP         = 4;
static constexpr int ENGINE_RAMP_DELAY_MS      = 10;
static constexpr int ENGINE_SPEED_LIMIT        = 255;

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
static constexpr uint32_t STATUS_INTERVAL_MS       = 50;

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
