#include "Process.h"
#include "config/Config.h"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace {
} // namespace

Process::Process(TelemetryEmitter *telemetry) : telemetry_(telemetry) {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData,
					 float lastSteerAngle, uint64_t tick, uint64_t scan_id,
					 const std::string &run_id) const {
	const uint64_t t0_us = mc::core::Time::us();
	(void)lastSteerAngle;
	static constexpr float kRadToDeg = 57.2957795f;
	std::array< int32_t, cfg::FTG_BIN_COUNT > bins{};
	std::array< float, cfg::FTG_BIN_COUNT > smoothed{};
	for (int i = 0; i < cfg::FTG_BIN_COUNT; ++i) {
		bins[(size_t)i] = 0;
		smoothed[(size_t)i] = 0.0f;
	}

	for (const auto &p : lidarData) {
		int angle = static_cast< int >(std::lround(p.angle));
		if (angle < cfg::FTG_ANGLE_MIN_DEG || angle > cfg::FTG_ANGLE_MAX_DEG)
			continue;
		if (p.distance <= 0)
			continue;
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		if (p.distance > bins[(size_t)idx])
			bins[(size_t)idx] = p.distance;
	}

	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
			 ++angle) {
		int sum = 0;
		int count = 0;
		for (int d = -cfg::FTG_SMOOTH_RADIUS_BINS;
			 d <= cfg::FTG_SMOOTH_RADIUS_BINS; ++d) {
			int a = angle + d;
			if (a < cfg::FTG_ANGLE_MIN_DEG)
				a = cfg::FTG_ANGLE_MIN_DEG;
			else if (a > cfg::FTG_ANGLE_MAX_DEG)
				a = cfg::FTG_ANGLE_MAX_DEG;
			const int idx = a - cfg::FTG_ANGLE_MIN_DEG;
			const int32_t dist = bins[(size_t)idx];
			if (dist > 0) {
				sum += dist;
				++count;
			}
		}
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		smoothed[(size_t)idx] = (count > 0) ? (float)sum / (float)count : 0.0f;
	}

	int best_angle = 0;
	float best_dist = -1.0f;
	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
			 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const float dist = smoothed[(size_t)idx];
		if (dist > best_dist) {
			best_dist = dist;
			best_angle = angle;
		} else if (dist == best_dist && std::abs(angle) < std::abs(best_angle)) {
			best_angle = angle;
		}
	}

	if (best_dist <= 0.0f) {
		const uint64_t ts_us = mc::core::Time::us();
		if (telemetry_)
			telemetry_->emitNoLidar(ts_us, run_id, tick, scan_id);
		return ProcResult(0, 0);
	}

	const float r_m = std::max(0.001f, best_dist / 1000.0f);
	const float theta_req_rad =
		2.0f *
		std::atan((cfg::FTG_CAR_WIDTH_M * 0.5f + cfg::FTG_MARGIN_M) / r_m);
	const float theta_req_deg = theta_req_rad * kRadToDeg;
	int n = static_cast< int >(std::ceil(theta_req_deg));
	if (n < 0)
		n = 0;
	const int max_n = std::max(-cfg::FTG_ANGLE_MIN_DEG, cfg::FTG_ANGLE_MAX_DEG);
	if (n > max_n)
		n = max_n;

	bool blocked = false;
	for (int angle = best_angle - n; angle <= best_angle + n; ++angle) {
		if (angle < cfg::FTG_ANGLE_MIN_DEG || angle > cfg::FTG_ANGLE_MAX_DEG)
			continue;
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const int32_t dist = bins[(size_t)idx];
		if (dist > 0 && dist <= cfg::FTG_NEAR_OBSTACLE_MM) {
			blocked = true;
			break;
		}
	}

	int out_angle = blocked ? 0 : best_angle;
	if (out_angle > cfg::STEER_ANGLE_MAX_DEG)
		out_angle = cfg::STEER_ANGLE_MAX_DEG;
	else if (out_angle < -cfg::STEER_ANGLE_MAX_DEG)
		out_angle = -cfg::STEER_ANGLE_MAX_DEG;

	int out_speed = 0;
	if (!blocked) {
		const float v_min = (float)cfg::FTG_SPEED_MIN;
		const float v_max = (float)cfg::FTG_SPEED_MAX;
		const float r_m = std::max(0.0f, best_dist / 1000.0f);
		float v_dist = v_min;
		if (r_m <= cfg::FTG_SPEED_R_SAFE_M) {
			v_dist = v_min;
		} else if (r_m >= cfg::FTG_SPEED_R_MAX_M) {
			v_dist = v_max;
		} else {
			v_dist = v_min + (v_max - v_min) *
							(1.0f - std::exp(-(r_m - cfg::FTG_SPEED_R_SAFE_M) /
											   cfg::FTG_SPEED_K_M));
		}

		const float steer_ratio =
			std::min(1.0f, std::fabs((float)out_angle) /
							 (float)cfg::STEER_ANGLE_MAX_DEG);
		static constexpr float kPiOver2 = 1.57079632679f;
		const float v_steer = v_min + (v_max - v_min) *
							 std::cos(steer_ratio * kPiOver2);

		const float v_final = std::min(v_dist, v_steer);
		const int speed = (int)std::lround(
			std::max(v_min, std::min(v_max, v_final)));
		out_speed = speed;
	}

	if (telemetry_) {
		TelemetrySample sample;
		sample.ts_us = mc::core::Time::us();
		sample.run_id = run_id;
		sample.tick = tick;
		sample.scan_id = scan_id;
		sample.best_angle_deg = (float)best_angle;
		sample.best_dist_mm = static_cast< int >(best_dist);
		sample.best_score = 1.0f;
		sample.best_delta_deg = std::nullopt;
		sample.min_handle_angle_deg = 0.0f;
		sample.min_handle_dist_mm = 0;
		sample.path_obst_mm = blocked ? cfg::FTG_NEAR_OBSTACLE_MM : 0;
		sample.front_dist_mm = std::nullopt;
		sample.side_dist_mm = std::nullopt;
		sample.base_speed = out_speed;
		sample.limited_speed = out_speed;
		sample.steer_deg = out_angle;
		sample.raw_steer_deg = (float)out_angle;
		sample.curve_ratio = 1.0f;
		sample.speed_factor = (out_speed > 0) ? 1.0f : 0.0f;
		sample.steer_clamp_deg = cfg::STEER_ANGLE_MAX_DEG;
		sample.override_kind = blocked ? "BLOCKED" : "NONE";
		sample.override_detail =
			blocked ? ("NEAR<=" +
					   std::to_string(cfg::FTG_NEAR_OBSTACLE_MM) + "mm")
					  : "NONE";
		sample.th_stop_mm = cfg::FTG_NEAR_OBSTACLE_MM;
		sample.th_safe_mm = cfg::FTG_NEAR_OBSTACLE_MM;
		sample.scan_age_ms = std::nullopt;
		const uint64_t t1_us = mc::core::Time::us();
		sample.planner_latency_ms = (uint32_t)((t1_us - t0_us) / 1000);
		sample.control_latency_ms = std::nullopt;
		sample.ttl_ms = cfg::AUTO_TTL_MS;
		sample.lidar_points = lidarData.size();
		sample.lidar_expected = cfg::FTG_BIN_COUNT;
		sample.include_candidates = false;
		telemetry_->emit(sample);
	}

	return ProcResult(out_speed, out_angle);
}
