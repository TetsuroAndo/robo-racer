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
		out.targets.speed_mm_s = st.target_speed_mm_s;
		out.targets.steer_cdeg = st.target_steer_cdeg;
		out.targets.ttl_ms = st.target_ttl_ms;
		out.targets.dist_mm = st.target_dist_mm;
		out.fresh = true;
		return out;
	}

	out.targets.speed_mm_s = 0;
	out.targets.steer_cdeg = 0;
	out.targets.ttl_ms = 100;
	out.targets.dist_mm = 0;
	out.fresh = false;
	return out;
}
