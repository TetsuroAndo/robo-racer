#include "AutoCommandSource.h"

AutoCommandResult AutoCommandSource::update(uint32_t now_ms,
											const mc::ControlState &st) const {
	AutoCommandResult out{};
	const bool auto_mode = (st.mode == mc::Mode::AUTO);
	const bool hb_fresh = auto_mode && (st.last_hb_ms != 0) &&
						  ((uint32_t)(now_ms - st.last_hb_ms) <=
						   (uint32_t)cfg::HEARTBEAT_TIMEOUT_MS);
	const bool cmd_fresh =
		auto_mode && (st.last_cmd_ms != 0) &&
		((uint32_t)(now_ms - st.last_cmd_ms) <= (uint32_t)st.target_ttl_ms);
	if (cmd_fresh && hb_fresh && !st.killed) {
		int16_t speed = st.target_speed_mm_s;
		if (speed <= 0)
			speed = (int16_t)cfg::CREEP_SPEED_MM_S;
		out.targets.speed_mm_s = speed;
		out.targets.steer_cdeg = st.target_steer_cdeg;
		out.targets.ttl_ms = st.target_ttl_ms;
		out.targets.dist_mm = st.target_dist_mm;
		out.fresh = true;
		return out;
	}

	out.targets.speed_mm_s = 0;
	out.targets.steer_cdeg = 0;
	out.targets.ttl_ms = cfg::DRIVE_TTL_DEFAULT_MS;
	out.targets.dist_mm = 0;
	out.fresh = false;
	return out;
}
