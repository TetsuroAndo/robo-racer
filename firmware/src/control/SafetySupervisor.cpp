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

	// 停止禁止: ブレーキ要求を無効化（PWM側で最低値に置換）
	if (out.targets.speed_mm_s <= 0) {
		out.stop_requested = false;
		out.stop_level = StopLevel::NONE;
	} else {
		out.stop_requested = d->tsd.stop_requested;
		out.stop_level = d->tsd.stop_level;
	}
	return out;
}
