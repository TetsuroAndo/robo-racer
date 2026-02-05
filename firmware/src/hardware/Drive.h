#pragma once
#include "Engine.h"
#include "Steer.h"
#include "../config/Config.h"
#include <Arduino.h>

class Drive {
public:
	void begin();

	void setTargetMmS(int16_t speed_mm_s);
	void setTargetSteerCdeg(int16_t steer_cdeg);
	void setTtlMs(uint16_t ttl_ms);
	void setDistMm(uint16_t dist_mm);

	void tick(uint32_t now_ms, float dt_s, bool killed);
	void setBrakeMode(bool enabled);

	int16_t appliedSpeedMmS() const { return _applied_speed_mm_s; }
	int16_t appliedSteerCdeg() const { return _applied_steer_cdeg; }
	uint16_t ttlMs() const { return _ttl_ms; }
	uint16_t distMm() const { return _dist_mm; }
	uint32_t lastUpdateMs() const { return _lastUpdateMs; }

private:
	Engine _engine;
	Steer  _steer;

	int16_t _tgt_speed_mm_s = 0;
	int16_t _tgt_steer_cdeg = 0;
	uint16_t _ttl_ms = 100;
	uint16_t _dist_mm = 0;

	int16_t _applied_speed_mm_s = 0;
	int16_t _applied_steer_cdeg = 0;

	uint32_t _lastUpdateMs = 0;

	static constexpr int MAX_SPEED_MM_S = cfg::DRIVE_SPEED_MAX_MM_S;
	bool _brake_mode = false;
	int speedMmSToPwm_(int16_t mm_s) const;
	float steerCdegToDeg_(int16_t cdeg) const;
};
