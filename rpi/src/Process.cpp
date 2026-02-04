#include "Process.h"
#include "config/Config.h"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
						 const std::string &run_id) const {
	const uint64_t t0_us = mc::core::Time::us();
	float max = 0;
	int maxDistance = -1;
	float minHandleAngle = 0;
	int minHandleDistance = INT32_MAX;
	int minObstacleOnPath = INT32_MAX;
	std::vector< CandidateScore > candidates;
	std::array< float, TELEMETRY_HEAT_BINS > heat_bins{};
	std::array< int, TELEMETRY_COMPASS_BINS > lidar_bins{};
	lidar_bins.fill(-1);
	bool lidar_bins_valid = false;
	for (auto &v : heat_bins)
		v = 0.0f;

	// 前回のステアリング角度を±cfg::STEER_ANGLE_MAX_DEGにクリップ
	float clampedSteerAngle = lastSteerAngle;
	if (clampedSteerAngle > cfg::STEER_ANGLE_MAX_DEG) {
		clampedSteerAngle = cfg::STEER_ANGLE_MAX_DEG;
	} else if (clampedSteerAngle < -cfg::STEER_ANGLE_MAX_DEG) {
		clampedSteerAngle = -cfg::STEER_ANGLE_MAX_DEG;
	}

	// クリップされたステアリング角度を中心とした評価範囲を計算
	float pathCheckMin = clampedSteerAngle - cfg::PROCESS_STEER_WINDOW_HALF_DEG;
	float pathCheckMax = clampedSteerAngle + cfg::PROCESS_STEER_WINDOW_HALF_DEG;

	// std::cout << lidarData.size() << "\n";
	for (const auto &i : lidarData) {
		// ハンドリング評価（±90deg）: 次の進行方向を決定
		if (cfg::PROCESS_HANDLE_ANGLE_MIN_DEG <= i.angle &&
			i.angle <= cfg::PROCESS_HANDLE_ANGLE_MAX_DEG) {
			const float ratio = (i.angle - cfg::PROCESS_HANDLE_ANGLE_MIN_DEG) /
								(cfg::PROCESS_HANDLE_ANGLE_MAX_DEG -
								 cfg::PROCESS_HANDLE_ANGLE_MIN_DEG);
			int idx =
				(int)std::lround(ratio * (float)(TELEMETRY_COMPASS_BINS - 1));
			if (idx < 0)
				idx = 0;
			if (idx >= (int)TELEMETRY_COMPASS_BINS)
				idx = (int)TELEMETRY_COMPASS_BINS - 1;
			if (lidar_bins[idx] < 0 || i.distance < lidar_bins[idx]) {
				lidar_bins[idx] = i.distance;
			}
			lidar_bins_valid = true;
			candidates.push_back(CandidateScore{.angle_deg = i.angle,
												.distance_mm = i.distance,
												.score = 0.0f});
			if (maxDistance < i.distance) {
				max = i.angle;
				maxDistance = i.distance;
			}
			if (i.distance < minHandleDistance) {
				minHandleAngle = i.angle;
				minHandleDistance = i.distance;
			}
		}

		// 進行経路上の障害物評価: 前回のステアリング角度±25度
		if (pathCheckMin <= i.angle && i.angle <= pathCheckMax) {
			if (i.distance < minObstacleOnPath) {
				minObstacleOnPath = i.distance;
			}
		}
	}

	// データが無い場合は安全側で停止
	if (maxDistance < 0) {
		const uint64_t ts_us = mc::core::Time::us();
		if (telemetry_)
			telemetry_->emitNoLidar(ts_us, run_id, tick, scan_id);
		return ProcResult(0, 0);
	}

	for (auto &c : candidates) {
		c.score = (maxDistance > 0) ? (float)c.distance_mm / (float)maxDistance
									: 0.0f;
		const float step = 180.0f / (float)(TELEMETRY_HEAT_BINS - 1);
		int idx = (int)std::round((c.angle_deg + 90.0f) / step);
		if (idx < 0)
			idx = 0;
		if (idx >= (int)TELEMETRY_HEAT_BINS)
			idx = (int)TELEMETRY_HEAT_BINS - 1;
		if (c.score > heat_bins[(size_t)idx])
			heat_bins[(size_t)idx] = c.score;
	}

	// 基本速度を計算
	int baseSpeed = maxDistance / cfg::PROCESS_SPEED_DIV;

	// 最大速度でクリップ
	baseSpeed = (baseSpeed > cfg::PROCESS_MAX_SPEED) ? cfg::PROCESS_MAX_SPEED
													 : baseSpeed;

	// 計算される角度（クリップ前）
	// best 角（最大距離）が無い場合は直進
	float steerSourceAngle = (maxDistance < 0) ? 0.0f : max;

	// 現在のステアリング方向との角度差に基づいた重み付け
	// 角度差が小さいほど、現在の方向をより優先
	if (minHandleDistance != INT32_MAX) {
		float angleDiff = std::abs(steerSourceAngle - clampedSteerAngle);
		// 角度差に基づいた重み（0°:1.0、90°:減少）
		float directionWeight =
			1.0f - (angleDiff / 90.0f) * cfg::PROCESS_DIRECTION_WEIGHT;
		// 現在の方向との平均を取ることで滑らかに遷移（角度差に応じて重みを変化）
		steerSourceAngle = steerSourceAngle * (1.0f - directionWeight) +
						   clampedSteerAngle * directionWeight;
	}

	float calculatedAngle = steerSourceAngle * cfg::PROCESS_MIN_ANGLE_SIGN *
							cfg::PROCESS_STEER_GAIN;
	float absAngle = std::fabs(calculatedAngle);
	float curve_ratio = 1.0f;

	// 進行経路上の障害物距離に基づいて速度を制限
	int limitedSpeed = baseSpeed;
	std::string override_reason = "NONE";
	if (minObstacleOnPath <= cfg::PROCESS_MIN_DIST_STOP_MM) {
		// 停止距離以下 → 停止
		limitedSpeed = 0;
		override_reason = "STOP";
	} else if (minObstacleOnPath < cfg::PROCESS_MIN_DIST_SAFE_MM) {
		// 安全距離未満 → 減速
		limitedSpeed = (int)(baseSpeed * cfg::PROCESS_MIN_DIST_SPEED_FACTOR);
		override_reason = "SLOW";
	}

	// 急カーブによる速度制限
	// 計算角度が物理上限を超える場合、その比率に応じて速度を減速
	if (absAngle > cfg::STEER_ANGLE_MAX_DEG) {
		curve_ratio = (float)cfg::STEER_ANGLE_MAX_DEG / absAngle;
		float curveFactor = cfg::STEER_CURVE_SPEED_FACTOR * curve_ratio;
		int curveReducedSpeed = (int)(limitedSpeed * curveFactor);
		limitedSpeed = curveReducedSpeed;
		if (override_reason == "NONE") {
			override_reason = "CURVE";
		} else {
			override_reason += "+CURVE";
		}
	}

	int roundedAngle = static_cast< int >(std::round(calculatedAngle));
	const float speed_factor =
		(baseSpeed > 0) ? (float)limitedSpeed / (float)baseSpeed : 0.0f;

	std::partial_sort(
		candidates.begin(),
		candidates.begin() + std::min< size_t >(3, candidates.size()),
		candidates.end(), [](const CandidateScore &a, const CandidateScore &b) {
			return a.score > b.score;
		});

	if (telemetry_) {
		std::optional< float > best_delta;
		if (has_last_best_) {
			best_delta = max - last_best_angle_;
		}
		last_best_angle_ = max;
		has_last_best_ = true;
		const bool include_candidates =
			(override_reason != "NONE") ||
			(best_delta &&
			 std::fabs(*best_delta) >= cfg::TELEMETRY_CANDIDATE_EVENT_DEG);

		std::string override_detail = override_reason;
		if (override_reason == "STOP") {
			override_detail =
				"STOP<=" + std::to_string(cfg::PROCESS_MIN_DIST_STOP_MM) + "mm";
		} else if (override_reason == "SLOW") {
			override_detail =
				"SLOW<" + std::to_string(cfg::PROCESS_MIN_DIST_SAFE_MM) + "mm";
		}
		if (override_reason.find("CURVE") != std::string::npos) {
			override_detail += " curve_ratio=" + std::to_string(curve_ratio);
		}

		TelemetrySample sample;
		sample.ts_us = mc::core::Time::us();
		sample.run_id = run_id;
		sample.tick = tick;
		sample.scan_id = scan_id;
		sample.best_delta_deg = best_delta;
		sample.include_candidates = include_candidates;
		sample.best_angle_deg = max;
		sample.best_dist_mm = maxDistance;
		sample.best_score = 1.0f;
		sample.score_obstacle = std::nullopt;
		sample.score_width = std::nullopt;
		sample.score_goal = std::nullopt;
		sample.score_curvature = std::nullopt;
		sample.score_stability = std::nullopt;
		sample.score_map_conf = std::nullopt;
		sample.min_handle_angle_deg = minHandleAngle;
		sample.min_handle_dist_mm = minHandleDistance;
		sample.path_obst_mm = minObstacleOnPath;
		sample.front_dist_mm = std::nullopt;
		sample.side_dist_mm = std::nullopt;
		sample.base_speed = baseSpeed;
		sample.limited_speed = limitedSpeed;
		sample.steer_deg = roundedAngle;
		sample.raw_steer_deg = calculatedAngle;
		sample.curve_ratio = curve_ratio;
		sample.speed_factor = speed_factor;
		sample.steer_clamp_deg = cfg::STEER_ANGLE_MAX_DEG;
		sample.override_kind = override_reason;
		sample.override_detail = override_detail;
		sample.th_stop_mm = cfg::PROCESS_MIN_DIST_STOP_MM;
		sample.th_safe_mm = cfg::PROCESS_MIN_DIST_SAFE_MM;
		sample.scan_age_ms = std::nullopt;
		const uint64_t t1_us = mc::core::Time::us();
		sample.planner_latency_ms = (uint32_t)((t1_us - t0_us) / 1000);
		sample.control_latency_ms = std::nullopt;
		sample.ttl_ms = cfg::AUTO_TTL_MS;
		sample.lidar_points = lidarData.size();
		sample.lidar_expected = 181;
		sample.heat_bins = heat_bins;
		sample.lidar_dist_bins = lidar_bins;
		sample.lidar_dist_valid = lidar_bins_valid;

		const size_t top_n = std::min< size_t >(3, candidates.size());
		sample.top_count = top_n;
		for (size_t i = 0; i < top_n; ++i) {
			sample.top[i] = {candidates[i].angle_deg, candidates[i].distance_mm,
							 candidates[i].score};
		}
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

	return ProcResult(limitedSpeed, roundedAngle);
}
