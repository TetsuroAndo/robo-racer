#pragma once
#include "Engine.h"
#include "Steer.h"
#include "../config/Config.h"
#include <Arduino.h>

class Drive {
public:
	void begin();

	void setTargetMmS(int16_t speed_mm_s, uint32_t now_ms);
	void setTargetPwm(int16_t speed_pwm, uint32_t now_ms);
	void setTargetBrake(uint8_t brake_duty, bool active, uint32_t now_ms);
	void setTargetSteerCdeg(int16_t steer_cdeg, uint32_t now_ms);
	void setTtlMs(uint16_t ttl_ms, uint32_t now_ms);
	void setDistMm(uint16_t dist_mm, uint32_t now_ms);

	void tick(uint32_t now_ms, float dt_s, bool killed);
	void setBrakeMode(bool enabled);

	int16_t appliedSpeedMmS() const { return _applied_speed_mm_s; }
	int16_t appliedPwm() const { return _applied_pwm; }
	uint8_t appliedBrakeDuty() const { return _applied_brake_duty; }
	int16_t appliedSteerCdeg() const { return _applied_steer_cdeg; }
	uint16_t ttlMs() const { return _ttl_ms; }
	uint16_t distMm() const { return _dist_mm; }
	uint32_t lastUpdateMs() const { return _lastUpdateMs; }

private:
	Engine _engine;
	Steer  _steer;

	int16_t _tgt_speed_mm_s = 0;
	int16_t _tgt_pwm = 0;
	uint8_t _tgt_brake_duty = 0;
	bool _brake_active = false;
	int16_t _tgt_steer_cdeg = 0;
	uint16_t _ttl_ms = 100;
	uint16_t _dist_mm = 0;

	int16_t _applied_speed_mm_s = 0;
	int16_t _applied_pwm = 0;
	uint8_t _applied_brake_duty = 0;
	int16_t _applied_steer_cdeg = 0;

	uint32_t _lastUpdateMs = 0;
	uint32_t _brake_released_at_ms = 0; // ブレーキ解除時刻（再加速抑制用）
	bool _prev_brake_active = false;     // 前回 tick でブレーキ中だったか

	bool _brake_mode = false;
	float steerCdegToDeg_(int16_t cdeg) const;
};
