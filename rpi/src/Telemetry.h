#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

struct TelemetryCandidate {
	float angle_deg{};
	int distance_mm{};
	float score{};
};

struct TelemetrySample {
	uint64_t ts_us{};
	std::string run_id;
	uint64_t tick{};
	uint64_t scan_id{};
	std::string mode{"UNKNOWN"};

	float best_angle_deg{};
	int best_dist_mm{};
	float best_score{};

	float min_handle_angle_deg{};
	int min_handle_dist_mm{};
	int path_obst_mm{};

	int base_speed{};
	int limited_speed{};
	int steer_deg{};
	int steer_clamp_deg{};

	std::string override_kind{"NONE"};
	int th_stop_mm{};
	int th_safe_mm{};

	std::array< TelemetryCandidate, 3 > top{};
	size_t top_count{};
};

class TelemetryEmitter {
public:
	void emit(const TelemetrySample &s);
	void emitNoLidar(uint64_t ts_us,
					 const std::string &run_id,
					 uint64_t tick,
					 uint64_t scan_id);
	void setUiEnabled(bool enabled) { ui_enabled_ = enabled; }
	void shutdownUi();

private:
	void emitJson_(const TelemetrySample &s);
	void emitOverrideEvent_(const TelemetrySample &s);
	void emitUi_(const TelemetrySample &s);

	std::string last_override_;
	bool ui_initialized_ = false;
	bool ui_enabled_ = true;
};
