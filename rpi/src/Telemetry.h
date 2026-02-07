#pragma once

#include "MotionState.h"
#include "Tsd20State.h"
#include "mc/core/Time.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

struct TelemetryCandidate {
	float angle_deg{};
	int distance_mm{};
	float score{};
};

enum class Severity : uint8_t { Safe = 0, Warn = 1, Crit = 2 };

static constexpr size_t TELEMETRY_HEAT_BINS = 37;
static constexpr size_t TELEMETRY_COMPASS_BINS = 61;

struct TelemetrySample {
	uint64_t ts_us{};
	std::string run_id;
	uint64_t tick{};
	uint64_t scan_id{};
	std::string mode{"UNKNOWN"};
	std::string map_state{"UNKNOWN"};
	std::optional< uint32_t > scan_age_ms;

	float best_angle_deg{};
	int best_dist_mm{};
	float best_score{};
	std::optional< float > best_delta_deg;

	float min_handle_angle_deg{};
	int min_handle_dist_mm{};
	int path_obst_mm{};
	std::optional< int > front_dist_mm;
	std::optional< int > side_dist_mm;

	int base_speed{};
	int limited_speed{};
	int steer_deg{};
	float raw_steer_deg{};
	float curve_ratio{1.0f};
	float speed_factor{1.0f};
	int steer_clamp_deg{};

	std::string override_kind{"NONE"};
	std::string override_detail{"NONE"};
	int th_stop_mm{};
	int th_safe_mm{};

	std::array< TelemetryCandidate, 3 > top{};
	size_t top_count{};
	std::vector< TelemetryCandidate > candidates;
	bool include_candidates = false;
	std::array< float, TELEMETRY_HEAT_BINS > heat_bins{};
	std::array< int, TELEMETRY_COMPASS_BINS > lidar_dist_bins{};
	bool lidar_dist_valid = false;

	std::optional< float > score_obstacle;
	std::optional< float > score_width;
	std::optional< float > score_goal;
	std::optional< float > score_curvature;
	std::optional< float > score_stability;
	std::optional< float > score_map_conf;

	std::optional< uint32_t > planner_latency_ms;
	std::optional< uint32_t > control_latency_ms;
	std::optional< uint16_t > ttl_ms;

	std::optional< size_t > lidar_points;
	std::optional< size_t > lidar_expected;
};

enum class TelemetryLevel : uint8_t { Basic = 0, Full = 1 };

class TelemetryEmitter {
public:
	TelemetryEmitter();
	~TelemetryEmitter();

	void emit(const TelemetrySample &s);
	void emitNoLidar(uint64_t ts_us,
					 const std::string &run_id,
					 uint64_t tick,
					 uint64_t scan_id);
	void setUiEnabled(bool enabled) { ui_enabled_ = enabled; }
	void setLevel(TelemetryLevel lv) { level_ = lv; }
	void setRateHz(double hz);
	void setMetricsLogPath(std::string path);
	void updateStatus(uint8_t auto_active,
					  uint16_t faults,
					  int16_t speed_mm_s,
					  int16_t steer_cdeg,
					  uint16_t age_ms,
					  uint8_t brake_duty = 0,
					  uint8_t stop_level = 0,
					  uint8_t stop_requested = 0);
	void updateDrops(uint32_t log_drop, uint32_t uart_drop);
	void updateMotion(const MotionState &motion);
	void updateTsd20(const Tsd20State &tsd);
	void shutdownUi();

private:
	enum class EventCategory : uint8_t {
		Ctrl = 0,
		Sens = 1,
		Map = 2,
		Loc = 3,
		Plan = 4,
		Override = 5,
		Count = 6,
	};
	struct EventState {
		bool valid = false;
		Severity severity = Severity::Safe;
		uint64_t ts_us = 0;
		uint32_t count = 0;
		std::string label;
	};

	void emitJson_(const TelemetrySample &s);
	void emitOverrideEvent_(const TelemetrySample &s);
	void emitUi_(const TelemetrySample &s);
	void refreshMetrics_();
	void metricsLoop_();
	void pushEvent_(EventCategory category,
					Severity severity,
					uint64_t ts_us,
					std::string label);

	struct MetricsCache {
		bool valid = false;
		uint16_t cpu_temp_cdeg = 0;
		uint16_t cpu_usage_permille = 0;
		uint32_t mem_used_kb = 0;
		uint32_t mem_total_kb = 0;
	};
	struct StatusCache {
		bool valid = false;
		uint8_t auto_active = 0;
		uint16_t faults = 0;
		int16_t speed_mm_s = 0;
		int16_t steer_cdeg = 0;
		uint16_t age_ms = 0;
		uint8_t brake_duty = 0;
		uint8_t stop_level = 0;
		uint8_t stop_requested = 0;
		uint32_t log_drop = 0;
		uint32_t uart_drop = 0;
		uint64_t ts_us = 0;
		bool drops_valid = false;
		uint64_t drops_ts_us = 0;
	};
	struct MotionCache {
		bool valid = false;
		bool calibrated = false;
		bool brake_mode = false;
		int16_t a_long_mm_s2 = 0;
		int16_t v_est_mm_s = 0;
		uint16_t a_brake_cap_mm_s2 = 0;
		float yaw_dps = 0.0f;
		uint16_t age_ms = 0;
		uint64_t ts_us = 0;
	};
	struct Tsd20Cache {
		bool valid = false;
		bool ready = false;
		bool sensor_valid = false;
		int32_t mm = 0;
		int32_t fails = 0;
		int32_t period_ms = 0;
		uint64_t ts_us = 0;
	};

	std::string last_override_;
	bool ui_initialized_ = false;
	bool ui_enabled_ = true;
	TelemetryLevel level_ = TelemetryLevel::Basic;
	uint64_t telemetry_interval_us_ = 100000;
	uint64_t last_telemetry_us_ = 0;
	uint64_t last_ui_us_ = 0;
	std::string metrics_log_path_;
	uint64_t metrics_last_read_us_ = 0;
	MetricsCache metrics_;
	StatusCache status_;
	MotionCache motion_;
	Tsd20Cache tsd20_;
	std::mutex metrics_mtx_;

	std::atomic< bool > running_{false};
	std::thread metrics_thread_;

	std::vector< float > spark_best_;
	std::vector< float > spark_speed_;
	std::vector< float > spark_dist_;
	std::array< EventState, static_cast< size_t >(EventCategory::Count) >
		event_state_{};
	bool has_last_delta_ = false;
	float last_delta_ = 0.0f;
	Severity last_loc_sev_ = Severity::Safe;
	Severity last_sens_sev_ = Severity::Safe;
	Severity last_ctrl_sev_ = Severity::Safe;
	Severity last_map_sev_ = Severity::Safe;
	bool last_best_jump_ = false;
	bool last_slowdown_ = false;
	bool last_ttl_expired_ = false;
	bool last_faults_ = false;
};

inline void TelemetryEmitter::updateStatus(uint8_t auto_active,
										   uint16_t faults,
										   int16_t speed_mm_s,
										   int16_t steer_cdeg,
										   uint16_t age_ms,
										   uint8_t brake_duty,
										   uint8_t stop_level,
										   uint8_t stop_requested) {
	std::lock_guard< std::mutex > lk(metrics_mtx_);
	status_.valid = true;
	status_.auto_active = auto_active;
	status_.faults = faults;
	status_.speed_mm_s = speed_mm_s;
	status_.steer_cdeg = steer_cdeg;
	status_.age_ms = age_ms;
	status_.brake_duty = brake_duty;
	status_.stop_level = stop_level;
	status_.stop_requested = stop_requested;
	status_.ts_us = mc::core::Time::us();
}

inline void TelemetryEmitter::updateDrops(uint32_t log_drop,
										  uint32_t uart_drop) {
	std::lock_guard< std::mutex > lk(metrics_mtx_);
	status_.log_drop = log_drop;
	status_.uart_drop = uart_drop;
	status_.drops_valid = true;
	status_.drops_ts_us = mc::core::Time::us();
}

inline void TelemetryEmitter::updateMotion(const MotionState &motion) {
	std::lock_guard< std::mutex > lk(metrics_mtx_);
	motion_.valid = motion.valid;
	motion_.calibrated = motion.calibrated;
	motion_.brake_mode = motion.brake_mode;
	motion_.a_long_mm_s2 = motion.a_long_mm_s2;
	motion_.v_est_mm_s = motion.v_est_mm_s;
	motion_.a_brake_cap_mm_s2 = motion.a_brake_cap_mm_s2;
	motion_.yaw_dps = motion.yaw_dps;
	motion_.age_ms = motion.age_ms;
	motion_.ts_us = mc::core::Time::us();
}

inline void TelemetryEmitter::updateTsd20(const Tsd20State &tsd) {
	std::lock_guard< std::mutex > lk(metrics_mtx_);
	tsd20_.valid = tsd.valid;
	tsd20_.ready = tsd.ready;
	tsd20_.sensor_valid = tsd.sensor_valid;
	tsd20_.mm = tsd.mm;
	tsd20_.fails = tsd.fails;
	tsd20_.period_ms = tsd.period_ms;
	// TSD20 はセンサ生成時刻を保持し、更新時刻系 (status/motion) と区別する。
	tsd20_.ts_us = tsd.ts_us;
}
