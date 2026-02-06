#pragma once

#include "StopLevel.h"
#include <stdint.h>

struct BrakeControllerOutput {
	// PWMを上書きする場合（逆転パルス / 惰行0固定）
	int16_t pwm_cmd = 0;
	bool pwm_override = false;

	// アクティブブレーキ（短絡制動）
	uint8_t brake_duty = 0;
	bool brake_active = false;

	// engine rate_down をブレーキ用にするか
	bool brake_mode = false;

	// デバッグ用（ログ可視化）
	uint8_t phase = 0;       // 0=IDLE, 1=REV, 2=COAST
	uint8_t pulse_count = 0;
	uint8_t pulse_limit = 0;
	uint16_t rev_on_ms = 0;
	// 適応ログ（REV終了時のみ。dv>0 で適応あり）
	float dv_mm_s = 0.0f;
	float a_eff_mm_s2 = 0.0f;
	float a_tgt_mm_s2 = 0.0f;
	float r_eff = 0.0f;
};

class BrakeController {
public:
	BrakeControllerOutput update(bool stop_requested, StopLevel stop_level,
								 uint16_t d_mm, float v_est_mm_s,
								 float a_brake_cap_mm_s2, uint32_t now_ms);

private:
	enum class Phase : uint8_t { IDLE = 0, REV = 1, COAST = 2 };
	Phase _phase = Phase::IDLE;
	uint32_t _phase_until_ms = 0;
	uint8_t _pulse_count = 0;
	uint8_t _pulse_limit = 0;
	uint16_t _rev_on_ms = 0;
	uint16_t _rev_on_ms_used = 0;
	float _rev_v_start = 0.0f;
	uint32_t _stop_since_ms = 0;
	uint32_t _last_stop_ms = 0;
	float _brake_ramp = 0.0f;
	StopLevel _latched_level = StopLevel::NONE;
	void reset_();
	void fillDebug_(BrakeControllerOutput& out, uint8_t phase) const;
};
