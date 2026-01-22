#pragma once

#include <stdint.h>

namespace cfg {
// clang-format off

//------------------------------------------------------------------------------
// シリアルログ出力（USB/UART0）
//------------------------------------------------------------------------------
static constexpr uint32_t LOG_BAUD      = 115200;

//------------------------------------------------------------------------------
// UART2（RPi ↔ ESP32）
//------------------------------------------------------------------------------

// ピン設定
static constexpr int      SERIAL_RX_PIN  = 16;
static constexpr int      SERIAL_TX_PIN  = 17;

// 通信設定
static constexpr uint32_t SERIAL_BAUD   = 921600;

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
static constexpr int      DRIVE_FRONT_AREA_DEG = 30;
static constexpr int      DRIVE_CURVE_AREA_DEG = 70;

// その他
static constexpr int      DRIVE_SPEED_MAX      = 255;
static constexpr int      DRIVE_AVE_DEG_NUM    = 1;

//------------------------------------------------------------------------------
// ステアサーボ設定（DS3218想定）
//------------------------------------------------------------------------------

// GPIO / PWM
static constexpr int STEER_PIN_SERVO           = 12;
static constexpr int STEER_CHANNEL             = 2;
static constexpr int STEER_PWM_FREQ_HZ         = 50;
static constexpr int STEER_PWM_RESOLUTION_BITS = 16;

// パルス幅
static constexpr int STEER_PULSE_MIN_US        = 1200;
static constexpr int STEER_PULSE_MAX_US        = 1800;
static constexpr int STEER_PULSE_CENTER_US     =
	(STEER_PULSE_MIN_US + STEER_PULSE_MAX_US) / 2;

// 角度マッピング
static constexpr int STEER_ANGLE_RANGE_DEG     = 30;
static constexpr int STEER_ANGLE_CENTER_DEG    = 90;

//------------------------------------------------------------------------------
// モータドライバ（IBT-2）とPWMランプ設定
//------------------------------------------------------------------------------

// GPIO
static constexpr int ENGINE_PIN_RPWM            = 25;
static constexpr int ENGINE_PIN_LPWM            = 26;
static constexpr int ENGINE_PIN_REN             = 27;
static constexpr int ENGINE_PIN_LEN             = 14;

// PWM
static constexpr int ENGINE_CHANNEL_RPWM        = 0;
static constexpr int ENGINE_CHANNEL_LPWM        = 1;
static constexpr int ENGINE_PWM_FREQ_HZ         = 20000;
static constexpr int ENGINE_PWM_RESOLUTION_BITS = 8;

// ランプ
static constexpr int ENGINE_SPEED_STEP          = 4;
static constexpr int ENGINE_RAMP_DELAY_MS       = 10;

//------------------------------------------------------------------------------
// 手動操作の上限（ゲームパッド）
//------------------------------------------------------------------------------

// 速度
static constexpr int   MANUAL_SPEED_MAX  = 180;
static constexpr int   MANUAL_SPEED_STEP = 6;

// 操舵
static constexpr float MANUAL_STEER_DEG  = 20.0f;

// clang-format on
} // namespace cfg
