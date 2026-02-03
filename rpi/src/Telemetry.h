#pragma once

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

static constexpr size_t TELEMETRY_HEAT_BINS = 37;

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
					  uint16_t age_ms);
	void shutdownUi();

private:
	void emitJson_(const TelemetrySample &s);
	void emitOverrideEvent_(const TelemetrySample &s);
	void emitUi_(const TelemetrySample &s);
	void refreshMetrics_();
	void metricsLoop_();
	void pushEvent_(std::string e);

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
	std::mutex metrics_mtx_;

	std::atomic< bool > running_{false};
	std::thread metrics_thread_;

	std::vector< float > spark_best_;
	std::vector< float > spark_speed_;
	std::vector< float > spark_dist_;
	std::vector< std::string > events_;
	bool has_last_delta_ = false;
	float last_delta_ = 0.0f;
};
