#include "SafetySupervisor.h"
#include "../config/Config.h"

SafetyResult SafetySupervisor::apply(uint32_t now_ms, float dt_s,
									 const Targets &desired, mc::Mode mode,
									 const Tsd20State &tsd,
									 const ImuEstimate &imu, bool imu_valid,
									 bool abs_allowed, SafetyDiag *diag) {
	SafetyDiag local{};
	SafetyDiag *d = diag ? diag : &local;
	*d = SafetyDiag{};

	SafetyResult out{};
	out.targets = desired;

	out.targets.speed_mm_s =
		_tsd.limit(out.targets.speed_mm_s, mode, tsd, imu, imu_valid,
				   out.targets.steer_cdeg, &d->tsd);

	// ABS は IMU が有効かつ、必要ならキャリブレーション完了時のみ許可する
	const bool imu_ok_for_abs =
		imu_valid && (!cfg::ABS_REQUIRE_CALIB || imu.calibrated);

	bool abs_active = false;
	out.targets.speed_mm_s = _abs.apply(now_ms, dt_s, out.targets.speed_mm_s,
										abs_allowed && imu_ok_for_abs, imu,
										imu_valid, tsd, &d->abs, &abs_active);
	out.brake_mode = abs_active;
	return out;
}
