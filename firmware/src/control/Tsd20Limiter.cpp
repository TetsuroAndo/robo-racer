#include "Tsd20Limiter.h"
#include "../../lib/common/Math.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>

int16_t Tsd20Limiter::limit(int16_t speed_mm_s, mc::Mode mode,
							const Tsd20State &tsd, const ImuEstimate &imu,
							bool imu_valid, int16_t steer_cdeg,
							Tsd20Diag *diag) const {
	Tsd20Diag local{};
	Tsd20Diag *d = diag ? diag : &local;
	*d = Tsd20Diag{};
	d->reason = TSD_DISABLED;
	d->clamped = false;
	// age_ms はここで一度だけ初期化し、以降の分岐では変更しない。
	d->age_ms = (tsd.age_ms != 0xFFFFu) ? (float)tsd.age_ms : -1.0f;
	// stop_requested/stop_level を初期化（diag が NULL でも安全）
	if (diag) {
		diag->stop_requested = false;
		diag->stop_level = StopLevel::NONE;
	}
	if (!cfg::TSD20_ENABLE)
		return speed_mm_s;

	if (speed_mm_s <= 0) {
		d->reason = TSD_NONPOSITIVE;
		return speed_mm_s;
	}
#if MC_ENABLE_MANUAL
	if (!cfg::TSD20_CLAMP_IN_MANUAL && mode != mc::Mode::AUTO) {
		d->reason = TSD_MANUAL_SKIP;
		return speed_mm_s;
	}
#endif
	if (cfg::TSD20_REQUIRE_OK && !tsd.ready) {
		d->reason = TSD_NOT_READY;
		return 0;
	}

	if (!tsd.valid) {
		d->reason = TSD_INVALID;
		if (tsd.fail_count >= cfg::TSD20_MAX_FAILS)
			return 0;
		return speed_mm_s;
	}

	if (tsd.age_ms != 0xFFFFu && tsd.age_ms > cfg::TSD20_MAX_AGE_MS) {
		d->reason = TSD_AGE_STALE;
		d->tau = -1.0f;
		d->v_max = 0.0f;
		d->v_cap = 0.0f;
		d->clamped = true;
		if (diag) {
			diag->stop_requested = true;
			diag->stop_level = StopLevel::STALE;
		}
		return 0;
	}

	if (cfg::TSD20_STOP_DISTANCE_MM > 0 &&
		tsd.mm <= cfg::TSD20_STOP_DISTANCE_MM) {
		d->reason = TSD_STOP;
		if (diag) {
			diag->stop_requested = true;
			diag->stop_level = StopLevel::STOP;
		}
		return 0;
	}

	if (tsd.mm <= cfg::TSD20_MARGIN_MM) {
		d->reason = TSD_MARGIN;
		if (diag) {
			diag->stop_requested = true;
			diag->stop_level = StopLevel::MARGIN;
		}
		return 0;
	}

	const float steer_abs = std::min((float)std::abs(steer_cdeg),
									 (float)mc_config::STEER_ANGLE_MAX_CDEG);
	const float steer_ratio =
		(mc_config::STEER_ANGLE_MAX_CDEG > 0)
			? (steer_abs / (float)mc_config::STEER_ANGLE_MAX_CDEG)
			: 0.0f;
	const float relax = (float)cfg::TSD20_MARGIN_RELAX_MM * steer_ratio;
	float base_margin = (float)cfg::TSD20_MARGIN_MM - relax;
	if (base_margin < (float)cfg::TSD20_MARGIN_MIN_MM)
		base_margin = (float)cfg::TSD20_MARGIN_MIN_MM;
	if (base_margin > (float)cfg::TSD20_MARGIN_MM)
		base_margin = (float)cfg::TSD20_MARGIN_MM;

	const bool imu_calibrated = imu_valid && imu.calibrated;
	const float a_min = imu_calibrated ? (float)cfg::IMU_BRAKE_MIN_MM_S2
									   : (float)cfg::IMU_BRAKE_MIN_UNCAL_MM_S2;
	const float a_max = imu_calibrated ? (float)cfg::IMU_BRAKE_MAX_MM_S2
									   : (float)cfg::IMU_BRAKE_MAX_UNCAL_MM_S2;
	float a = 0.0f;
	if (imu_calibrated && imu.a_brake_cap_mm_s2 > 0.0f)
		a = imu.a_brake_cap_mm_s2;
	if (a <= 0.0f) {
		a = imu_calibrated ? (float)cfg::IMU_BRAKE_INIT_MM_S2
						   : (float)cfg::IMU_BRAKE_INIT_UNCAL_MM_S2;
	}
	if (a < a_min)
		a = a_min;
	else if (a > a_max)
		a = a_max;

	// ベースとなるシステム遅延（センサ内部＋配線遅延など）
	const float tau_base = (float)cfg::TSD20_LATENCY_MS / 1000.0f;
	// 実際の鮮度 (Age) を秒に変換し、ベース遅延に加算する。
	float tau = tau_base;
	if (tsd.age_ms != 0xFFFFu) {
		const float age_s = (float)tsd.age_ms / 1000.0f;
		tau += age_s;
	}
	const float d_allow_base = (float)tsd.mm - base_margin;
	float margin_pred = 0.0f;
	if (cfg::TSD20_PREDICT_ENABLE && d_allow_base > 0.0f) {
		if (imu_valid && imu.calibrated) {
			const float v_est = std::max(0.0f, imu.v_est_mm_s);
			const float v_cmd = std::max(0.0f, (float)speed_mm_s);
			const float v_used = std::max(v_est, v_cmd);
			const float a_long = imu.a_long_mm_s2;
			const float a_cap = std::max(
				0.0f,
				std::min(a_long, (float)cfg::TSD20_PREDICT_ACCEL_MAX_MM_S2));
			const float d_travel = v_used * tau + 0.5f * a_cap * tau * tau;
			margin_pred = d_travel;
			if (margin_pred > (float)cfg::TSD20_PREDICT_MARGIN_MAX_MM)
				margin_pred = (float)cfg::TSD20_PREDICT_MARGIN_MAX_MM;
			d->v_est = v_used;
			d->a_long = a_long;
			d->d_travel = d_travel;
		}
	}
	const float margin_eff = base_margin + margin_pred;
	const float d_allow = (float)tsd.mm - margin_eff;
	const float term = a * tau;
	const float disc = term * term + 2.0f * a * d_allow;
	float v_max = (disc > 0.0f) ? (-term + std::sqrt(disc)) : 0.0f;
	if (v_max < 0.0f)
		v_max = 0.0f;

	float v_cap = std::min(v_max, (float)mc_config::SPEED_MAX_MM_S);
	const uint16_t stop_mm = cfg::TSD20_STOP_DISTANCE_MM;
	const uint16_t slow_mm = cfg::TSD20_SLOWDOWN_DISTANCE_MM;
	if (slow_mm > stop_mm && tsd.mm < slow_mm) {
		const float ratio =
			(float)(tsd.mm - stop_mm) / (float)(slow_mm - stop_mm);
		const float slow = mc::clamp< float >(ratio, 0.0f, 1.0f);
		v_cap *= slow;
	}

	// 4m未満なら「PWMスケール128相当」の速度(mm/s)まで上限を被せる
	// ここでは speed_mm_s 自体は変えず、最終cap(v_cap)にだけ反映する
	if (speed_mm_s > 0 && tsd.mm < cfg::TSD20_NEAR_DISTANCE_MM) {
		const float safe_engine_limit = (cfg::ENGINE_SPEED_LIMIT > 0)
											? (float)cfg::ENGINE_SPEED_LIMIT
											: 1.0f;
		const float v_near = (float)mc_config::SPEED_MAX_MM_S *
							 ((float)cfg::TSD20_NEAR_PWM_CAP / safe_engine_limit);
		v_cap = std::min(v_cap, v_near);
	}

	d->d_allow = d_allow;
	d->margin_eff = margin_eff;
	d->margin_pred = margin_pred;
	d->steer_ratio = steer_ratio;
	d->a_cap = a;
	d->tau = tau;
	d->v_max = v_max;
	d->v_cap = v_cap;
	d->clamped = ((float)speed_mm_s > v_cap);
	d->reason = d->clamped ? TSD_CLAMP : TSD_OK;
	if ((float)speed_mm_s > v_cap)
		return (int16_t)std::lround(v_cap);
	return speed_mm_s;
}
