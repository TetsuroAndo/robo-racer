#include "AbsController.h"
#include "../config/Config.h"
#include <algorithm>

void AbsController::reset(uint32_t /*now_ms*/) {
	// 簡略化後は状態保持なし
}

int16_t AbsController::apply(uint32_t /*now_ms*/, float dt_s,
							 int16_t speed_mm_s, bool allow_abs,
							 const ImuEstimate &imu, bool imu_valid,
							 const Tsd20State &tsd, AbsDiag *diag,
							 bool *active_out) {
	AbsDiag local{};
	AbsDiag *d = diag ? diag : &local;
	*d = AbsDiag{};
	d->reason = ABS_DISABLED;
	d->v_cmd = (float)speed_mm_s;
	d->v_est = imu_valid ? imu.v_est_mm_s : 0.0f;
	d->dt_s = dt_s;

	if (active_out)
		*active_out = false;

	if (!cfg::ABS_ENABLE || !allow_abs) {
		return speed_mm_s;
	}

	if (!imu_valid) {
		d->reason = ABS_NO_IMU;
		return speed_mm_s;
	}

	if (cfg::ABS_REQUIRE_CALIB && !imu.calibrated) {
		d->reason = ABS_NOT_CALIB;
		return speed_mm_s;
	}

	if (speed_mm_s < 0) {
		d->reason = ABS_REVERSE;
		return speed_mm_s;
	}

	const float v_cmd = std::max(0.0f, (float)speed_mm_s);
	const float v_est = std::max(0.0f, imu.v_est_mm_s);
	d->v_cmd = v_cmd;
	d->v_est = v_est;

	// 前進していないならABS無効
	if (v_est <= 0.0f) {
		d->reason = ABS_INACTIVE;
		return speed_mm_s;
	}

	// 目標が十分あるときは発動しない（減速したい状況のみ）
	if (v_cmd >= (float)cfg::ABS_V_CMD_MAX_MM_S) {
		d->reason = ABS_INACTIVE;
		return speed_mm_s;
	}

	// 微小速度は無視（ノイズ・手動移動の誤検出防止）
	if (v_est < (float)cfg::ABS_V_EST_MIN_MM_S) {
		d->reason = ABS_INACTIVE;
		return speed_mm_s;
	}

	// TSD20必須: 障害物が近いときのみ発動（壁なし誤発動防止）
	if (!cfg::TSD20_ENABLE || !tsd.valid || tsd.mm > cfg::ABS_TSD_TRIGGER_MM) {
		d->reason = ABS_INACTIVE;
		return speed_mm_s;
	}

	// 減速したいか
	const float margin = (float)cfg::ABS_SPEED_MARGIN_MM_S;
	const bool want_brake = (v_est > (v_cmd + margin));
	if (!want_brake) {
		d->reason = ABS_INACTIVE;
		return speed_mm_s;
	}

	// 惰行のみ（逆転なし）、brake_mode でブレーキ挙動
	d->reason = ABS_ACTIVE;
	d->active = true;
	if (active_out)
		*active_out = true;
	return 0;
}
