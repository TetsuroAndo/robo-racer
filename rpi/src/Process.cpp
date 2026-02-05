#include "Process.h"
#include "config/Config.h"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {
struct CandidateScore {
	float angle_deg{};
	int distance_mm{};
	float score{};
};
} // namespace

Process::Process(TelemetryEmitter *telemetry) : telemetry_(telemetry) {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData,
						 float lastSteerAngle, uint64_t tick, uint64_t scan_id,
						 const std::string &run_id,
						 const MotionState *motion) const {
	const uint64_t t0_us = mc::core::Time::us();
	static constexpr float kRadToDeg = 57.2957795f;
	const float max_steer = (float)cfg::STEER_ANGLE_MAX_DEG;
	float yaw_bias = 0.0f;
	if (motion && motion->valid && motion->age_ms <= cfg::FTG_IMU_MAX_AGE_MS &&
		cfg::FTG_YAW_BIAS_DEG > 0.0f && cfg::FTG_YAW_BIAS_REF_DPS > 0.0f) {
		float norm = motion->yaw_dps / cfg::FTG_YAW_BIAS_REF_DPS;
		if (norm > 1.0f)
			norm = 1.0f;
		else if (norm < -1.0f)
			norm = -1.0f;
		yaw_bias = cfg::FTG_YAW_BIAS_DEG * norm;
	}
	const float clamped_last =
		std::max(-max_steer, std::min(max_steer, lastSteerAngle + yaw_bias));
	float dt_s = 0.1f;
	if (last_proc_ts_us_ > 0 && t0_us > last_proc_ts_us_) {
		dt_s = (float)(t0_us - last_proc_ts_us_) / 1000000.0f;
		if (dt_s < 0.001f)
			dt_s = 0.001f;
		else if (dt_s > 0.5f)
			dt_s = 0.5f;
	}
	last_proc_ts_us_ = t0_us;
	std::array< int32_t, cfg::FTG_BIN_COUNT > bins{};
	std::array< float, cfg::FTG_BIN_COUNT > smoothed{};
	std::array< int32_t, cfg::FTG_BIN_COUNT > corridor_min{};
	for (int i = 0; i < cfg::FTG_BIN_COUNT; ++i) {
		bins[(size_t)i] = 0;
		smoothed[(size_t)i] = 0.0f;
		corridor_min[(size_t)i] = 0;
	}

	for (const auto &p : lidarData) {
		int angle = static_cast< int >(std::lround(p.angle));
		if (angle < cfg::FTG_ANGLE_MIN_DEG || angle > cfg::FTG_ANGLE_MAX_DEG)
			continue;
		if (p.distance <= 0)
			continue;
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		if (bins[(size_t)idx] == 0 || p.distance < bins[(size_t)idx])
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

	const int max_n = std::max(-cfg::FTG_ANGLE_MIN_DEG, cfg::FTG_ANGLE_MAX_DEG);
	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
		 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const float dist = smoothed[(size_t)idx];
		if (dist <= 0.0f)
			continue;
		float r_m = std::max(0.001f, dist / 1000.0f);
		if (cfg::FTG_CORRIDOR_LOOKAHEAD_M > 0.0f) {
			r_m = std::min(r_m, cfg::FTG_CORRIDOR_LOOKAHEAD_M);
		}
		const float theta_req_rad =
			2.0f *
			std::atan((cfg::FTG_CAR_WIDTH_M * 0.5f + cfg::FTG_MARGIN_M) / r_m);
		const float theta_req_deg = theta_req_rad * kRadToDeg;
		int n = static_cast< int >(std::ceil(theta_req_deg));
		if (n < 0)
			n = 0;
		if (n > max_n)
			n = max_n;
		int min_mm = std::numeric_limits< int32_t >::max();
		for (int a = angle - n; a <= angle + n; ++a) {
			if (a < cfg::FTG_ANGLE_MIN_DEG || a > cfg::FTG_ANGLE_MAX_DEG)
				continue;
			const int a_idx = a - cfg::FTG_ANGLE_MIN_DEG;
			const int32_t raw = bins[(size_t)a_idx];
			if (raw > 0 && raw < min_mm)
				min_mm = raw;
		}
		if (min_mm != std::numeric_limits< int32_t >::max())
			corridor_min[(size_t)idx] = min_mm;
	}

	int best_angle = 0;
	int best_dist = 0;
	float best_j = std::numeric_limits< float >::infinity();
	float z = 0.0f;
	float angle_sum = 0.0f;
	bool has_data = false;
	int min_corridor = std::numeric_limits< int32_t >::max();
	int min_angle = 0;
	auto obs_cost = [](int d_mm) -> float {
		if (d_mm >= cfg::FTG_COST_SAFE_MM)
			return 0.0f;
		const float x = (float)(cfg::FTG_COST_SAFE_MM - d_mm) /
						(float)cfg::FTG_COST_SAFE_MM;
		return x * x;
	};
	auto jerk_weight = [](int d_mm) -> float {
		float w = cfg::FTG_COST_W_DELTA;
		if (cfg::FTG_JERK_RELAX_MM > cfg::FTG_NEAR_OBSTACLE_MM) {
			if (d_mm < cfg::FTG_JERK_RELAX_MM) {
				float s =
					(float)(d_mm - cfg::FTG_NEAR_OBSTACLE_MM) /
					(float)(cfg::FTG_JERK_RELAX_MM - cfg::FTG_NEAR_OBSTACLE_MM);
				if (s < 0.0f)
					s = 0.0f;
				else if (s > 1.0f)
					s = 1.0f;
				w *= (s * s);
			}
		}
		return w;
	};
	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
		 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const int d_mm = corridor_min[(size_t)idx];
		if (d_mm <= 0)
			continue;
		has_data = true;
		if (d_mm < min_corridor) {
			min_corridor = d_mm;
			min_angle = angle;
		}
		if (d_mm <= cfg::FTG_NEAR_OBSTACLE_MM)
			continue;
		const float a_norm =
			std::fabs((float)angle) / (float)cfg::STEER_ANGLE_MAX_DEG;
		const float d_norm = std::fabs((float)angle - clamped_last) /
							 (float)cfg::STEER_ANGLE_MAX_DEG;
		const float w_delta = jerk_weight(d_mm);
		const float j = cfg::FTG_COST_W_OBS * obs_cost(d_mm) +
						cfg::FTG_COST_W_TURN * (a_norm * a_norm) +
						w_delta * (d_norm * d_norm);
		if (j < best_j) {
			best_j = j;
			best_angle = angle;
			best_dist = d_mm;
		}
		const float w = std::exp(-cfg::FTG_COST_BETA * j);
		z += w;
		angle_sum += w * (float)angle;
	}

	if (!has_data) {
		const uint64_t ts_us = mc::core::Time::us();
		if (telemetry_)
			telemetry_->emitNoLidar(ts_us, run_id, tick, scan_id);
		return ProcResult(0, 0);
	}

	const bool blocked = !(z > 0.0f);
	float target_angle_f = blocked ? 0.0f : (angle_sum / z);
	if (target_angle_f > cfg::STEER_ANGLE_MAX_DEG)
		target_angle_f = (float)cfg::STEER_ANGLE_MAX_DEG;
	else if (target_angle_f < -cfg::STEER_ANGLE_MAX_DEG)
		target_angle_f = (float)-cfg::STEER_ANGLE_MAX_DEG;

	const float max_delta = cfg::FTG_STEER_SLEW_DEG_PER_S * dt_s;
	float applied_angle_f = target_angle_f;
	const float delta = applied_angle_f - clamped_last;
	if (delta > max_delta)
		applied_angle_f = clamped_last + max_delta;
	else if (delta < -max_delta)
		applied_angle_f = clamped_last - max_delta;

	int out_angle = static_cast< int >(std::lround(applied_angle_f));
	if (out_angle > cfg::STEER_ANGLE_MAX_DEG)
		out_angle = cfg::STEER_ANGLE_MAX_DEG;
	else if (out_angle < -cfg::STEER_ANGLE_MAX_DEG)
		out_angle = -cfg::STEER_ANGLE_MAX_DEG;

	int corridor_min_mm = 0;
	if (blocked) {
		out_angle = 0;
		applied_angle_f = 0.0f;
		corridor_min_mm =
			(min_corridor == std::numeric_limits< int32_t >::max())
				? 0
				: min_corridor;
		best_angle = min_angle;
		best_dist = corridor_min_mm;
	} else {
		const int out_idx = out_angle - cfg::FTG_ANGLE_MIN_DEG;
		if (out_idx >= 0 && out_idx < cfg::FTG_BIN_COUNT) {
			corridor_min_mm = corridor_min[(size_t)out_idx];
		}
		if (corridor_min_mm <= 0) {
			out_angle = best_angle;
			applied_angle_f = (float)best_angle;
			corridor_min_mm = best_dist;
		}
	}
	const bool warn =
		(corridor_min_mm > 0) && (corridor_min_mm < cfg::FTG_WARN_OBSTACLE_MM);

	int out_speed = 0;
	if (!blocked) {
		int d_speed_mm = corridor_min_mm;
		const int out_idx = out_angle - cfg::FTG_ANGLE_MIN_DEG;
		if (out_idx >= 0 && out_idx < cfg::FTG_BIN_COUNT) {
			const int smoothed_mm =
				static_cast< int >(std::lround(smoothed[(size_t)out_idx]));
			if (smoothed_mm > 0)
				d_speed_mm = smoothed_mm;
		}

		const float v_min = (float)cfg::FTG_SPEED_MIN;
		const float v_max = (float)cfg::FTG_SPEED_MAX;
		const float r_m = std::max(0.0f, d_speed_mm / 1000.0f);
		float v_dist = v_min;
		if (r_m <= cfg::FTG_SPEED_R_SAFE_M) {
			v_dist = v_min;
		} else if (r_m >= cfg::FTG_SPEED_R_MAX_M) {
			v_dist = v_max;
		} else {
			v_dist =
				v_min + (v_max - v_min) *
							(1.0f - std::exp(-(r_m - cfg::FTG_SPEED_R_SAFE_M) /
											 cfg::FTG_SPEED_K_M));
		}

		const float steer_ratio =
			std::min(1.0f, std::fabs((float)out_angle) /
							   (float)cfg::STEER_ANGLE_MAX_DEG);
		static constexpr float kPiOver2 = 1.57079632679f;
		const float v_steer =
			v_min + (v_max - v_min) * std::cos(steer_ratio * kPiOver2);

		const float v_final = std::min(v_dist, v_steer);
		const int speed =
			(int)std::lround(std::max(v_min, std::min(v_max, v_final)));
		out_speed = speed;
		if (warn) {
			out_speed = std::min(out_speed, cfg::FTG_SPEED_WARN_CAP);
		}
	}

	if (telemetry_) {
		std::array< float, TELEMETRY_HEAT_BINS > heat_bins{};
		std::array< int, TELEMETRY_COMPASS_BINS > lidar_bins{};
		bool lidar_bins_valid = false;
		heat_bins.fill(0.0f);
		lidar_bins.fill(-1);

		std::vector< CandidateScore > candidates;
		candidates.reserve(cfg::FTG_BIN_COUNT);
		const float step = 180.0f / (float)(TELEMETRY_HEAT_BINS - 1);
		const float ratio_max =
			(cfg::FTG_ANGLE_MAX_DEG - cfg::FTG_ANGLE_MIN_DEG) > 0
				? 1.0f /
					  (float)(cfg::FTG_ANGLE_MAX_DEG - cfg::FTG_ANGLE_MIN_DEG)
				: 0.0f;
		float max_score = 0.0f;
		for (int angle = cfg::FTG_ANGLE_MIN_DEG;
			 angle <= cfg::FTG_ANGLE_MAX_DEG; ++angle) {
			const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
			const int d_mm = corridor_min[(size_t)idx];
			if (d_mm <= 0)
				continue;
			const float a_norm =
				std::fabs((float)angle) / (float)cfg::STEER_ANGLE_MAX_DEG;
			const float d_norm = std::fabs((float)angle - clamped_last) /
								 (float)cfg::STEER_ANGLE_MAX_DEG;
			const float w_delta = jerk_weight(d_mm);
			const float j = cfg::FTG_COST_W_OBS * obs_cost(d_mm) +
							cfg::FTG_COST_W_TURN * (a_norm * a_norm) +
							w_delta * (d_norm * d_norm);
			const float score = 1.0f / (1.0f + j);
			if (score > max_score)
				max_score = score;
			candidates.push_back(CandidateScore{(float)angle, d_mm, score});

			const float ratio =
				(float)(angle - cfg::FTG_ANGLE_MIN_DEG) * ratio_max;
			int compass_idx =
				(int)std::lround(ratio * (float)(TELEMETRY_COMPASS_BINS - 1));
			if (compass_idx < 0)
				compass_idx = 0;
			if (compass_idx >= (int)TELEMETRY_COMPASS_BINS)
				compass_idx = (int)TELEMETRY_COMPASS_BINS - 1;
			const int32_t raw_dist = bins[(size_t)idx];
			if (raw_dist > 0 && (lidar_bins[(size_t)compass_idx] < 0 ||
								 raw_dist < lidar_bins[(size_t)compass_idx])) {
				lidar_bins[(size_t)compass_idx] = raw_dist;
				lidar_bins_valid = true;
			}
		}
		if (max_score > 0.0f) {
			for (const auto &c : candidates) {
				int heat_idx = (int)std::round((c.angle_deg + 90.0f) / step);
				if (heat_idx < 0)
					heat_idx = 0;
				if (heat_idx >= (int)TELEMETRY_HEAT_BINS)
					heat_idx = (int)TELEMETRY_HEAT_BINS - 1;
				const float norm = c.score / max_score;
				if (norm > heat_bins[(size_t)heat_idx])
					heat_bins[(size_t)heat_idx] = norm;
			}
		}

		std::optional< float > best_delta;
		if (has_last_best_) {
			best_delta = applied_angle_f - last_best_angle_;
		}
		last_best_angle_ = applied_angle_f;
		has_last_best_ = true;

		std::string override_kind = "NONE";
		std::string override_detail = "NONE";
		if (corridor_min_mm <= cfg::FTG_NEAR_OBSTACLE_MM) {
			override_kind = "STOP";
			override_detail =
				"STOP<=" + std::to_string(cfg::FTG_NEAR_OBSTACLE_MM) + "mm";
		} else if (warn) {
			override_kind = "SLOW";
			override_detail =
				"SLOW<" + std::to_string(cfg::FTG_WARN_OBSTACLE_MM) + "mm";
		}
		const bool include_candidates =
			(override_kind != "NONE") ||
			(best_delta &&
			 std::fabs(*best_delta) >= cfg::TELEMETRY_CANDIDATE_EVENT_DEG);

		TelemetrySample sample;
		sample.ts_us = mc::core::Time::us();
		sample.run_id = run_id;
		sample.tick = tick;
		sample.scan_id = scan_id;
		sample.best_angle_deg = (float)best_angle;
		sample.best_dist_mm = best_dist;
		sample.best_score =
			std::isfinite(best_j) ? (1.0f / (1.0f + best_j)) : 0.0f;
		sample.best_delta_deg = best_delta;
		sample.min_handle_angle_deg = 0.0f;
		sample.min_handle_dist_mm = 0;
		sample.path_obst_mm = corridor_min_mm;
		sample.front_dist_mm = std::nullopt;
		sample.side_dist_mm = std::nullopt;
		sample.base_speed = out_speed;
		sample.limited_speed = out_speed;
		sample.steer_deg = out_angle;
		sample.raw_steer_deg = target_angle_f;
		sample.curve_ratio = 1.0f;
		sample.speed_factor = (out_speed > 0) ? 1.0f : 0.0f;
		sample.steer_clamp_deg = cfg::STEER_ANGLE_MAX_DEG;
		sample.override_kind = override_kind;
		sample.override_detail = override_detail;
		sample.th_stop_mm = cfg::FTG_NEAR_OBSTACLE_MM;
		sample.th_safe_mm = cfg::FTG_WARN_OBSTACLE_MM;
		sample.scan_age_ms = std::nullopt;
		const uint64_t t1_us = mc::core::Time::us();
		sample.planner_latency_ms = (uint32_t)((t1_us - t0_us) / 1000);
		if (motion && motion->valid &&
			motion->age_ms <= cfg::FTG_IMU_MAX_AGE_MS) {
			sample.control_latency_ms = motion->age_ms;
		} else {
			sample.control_latency_ms = std::nullopt;
		}
		sample.ttl_ms = cfg::AUTO_TTL_MS;
		sample.lidar_points = lidarData.size();
		sample.lidar_expected = cfg::FTG_BIN_COUNT;
		sample.heat_bins = heat_bins;
		sample.lidar_dist_bins = lidar_bins;
		sample.lidar_dist_valid = lidar_bins_valid;

		const size_t top_n = std::min< size_t >(3, candidates.size());
		sample.top_count = top_n;
		if (top_n > 0) {
			std::partial_sort(
				candidates.begin(), candidates.begin() + top_n,
				candidates.end(),
				[](const CandidateScore &a, const CandidateScore &b) {
					return a.score > b.score;
				});
			for (size_t i = 0; i < top_n; ++i) {
				sample.top[i] = {candidates[i].angle_deg,
								 candidates[i].distance_mm,
								 candidates[i].score};
			}
		}
		sample.include_candidates = include_candidates;
		if (include_candidates) {
			sample.candidates.clear();
			sample.candidates.reserve(candidates.size());
			for (const auto &c : candidates) {
				sample.candidates.push_back(
					{c.angle_deg, c.distance_mm, c.score});
			}
		}
		telemetry_->emit(sample);
	}

	return ProcResult(out_speed, out_angle);
}
