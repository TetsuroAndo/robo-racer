#include "SafetySupervisor.h"
#include "../config/Config.h"
#include "Tsd20Limiter.h"

SafetyResult SafetySupervisor::apply(uint32_t now_ms, float dt_s,
									 const Targets &desired, mc::Mode mode,
									 const Tsd20State &tsd,
									 const ImuEstimate &imu, bool imu_valid,
									 SafetyDiag *diag) {
	SafetyDiag local{};
	SafetyDiag *d = diag ? diag : &local;
	*d = SafetyDiag{};

	SafetyResult out{};
	out.targets = desired;

	out.targets.speed_mm_s =
		_tsd.limit(out.targets.speed_mm_s, mode, tsd, imu, imu_valid,
				   out.targets.steer_cdeg, &d->tsd);

	// TSD20 が STOP/MARGIN/AGE_STALE で速度0にしたとき BrakeController 用に
	// stop_requested（Tsd20Limiter 内で設定済み）
	out.stop_requested = d->tsd.stop_requested;
	out.stop_level = d->tsd.stop_level;
	return out;
}
