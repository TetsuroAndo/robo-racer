#pragma once
#include <stdint.h>

namespace mc::cfg {

// =======================
// Board & Bus
// =======================
static const uint32_t SERIAL_BAUD = 115200;

// I2C pins (ESP32 default: SDA=21, SCL=22; adjust if you rewired)
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;
static const uint32_t I2C_FREQ_HZ = 400000;

// =======================
// Actuators
// =======================
// IBT-2 / BTS7960 pins
static const int PIN_RPWM = 25;  // PWM forward
static const int PIN_LPWM = 26;  // PWM reverse
static const int PIN_REN  = 27;  // enable forward side (HIGH enable)
static const int PIN_LEN  = 14;  // enable reverse side (HIGH enable)

// Servo signal pin
static const int PIN_SERVO = 18;

// =======================
// Sensors
// =======================
static const uint8_t TSD20_I2C_ADDR_7BIT = 0x52; // per TSD20 manual
static const uint8_t MPU6050_I2C_ADDR_7BIT = 0x68;

// =======================
// Control Params (safe defaults; tune on carpet)
// =======================
static const float THROTTLE_MAX = 0.35f;       // [-1..1], keep low for first tests
static const float THROTTLE_REVERSE = -0.20f;  // gentle reverse
static const float THROTTLE_SLEW_UP = 1.0f;    // per second
static const float THROTTLE_SLEW_DN = 2.5f;    // per second

// Steering normalization [-1..1]
static const float STEER_CENTER = 0.0f;
static const float STEER_MAX = 1.0f;

// Straight heading hold (P controller on yaw error)
static const float YAW_KP = 0.020f; // steering per deg

// Obstacle thresholds (mm)
static const uint16_t DIST_EMERGENCY_STOP_MM = 200;
static const uint16_t DIST_SLOW_MM = 450;
static const uint16_t DIST_CLEAR_MM = 700;

// Turn maneuver
static const float TURN_TARGET_DEG = 55.0f;  // yaw delta to finish a turn
static const uint32_t TURN_BRAKE_MS = 150;
static const uint32_t TURN_REVERSE_MS = 250;
static const float TURN_THROTTLE = 0.22f;

static const bool PREFER_RIGHT_TURN_DEFAULT = true;

// RPi watchdog
static const uint32_t HOST_HEARTBEAT_TIMEOUT_MS = 700;

} // namespace mc::cfg
