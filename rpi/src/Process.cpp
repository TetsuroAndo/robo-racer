#include "Process.h"
#include "config/Config.h"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <cmath>
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
	float max = 0;
	int maxDistance = -1;
	float minHandleAngle = 0;
	int minHandleDistance = INT32_MAX;
	int minObstacleOnPath = INT32_MAX;
	std::vector< CandidateScore > candidates;

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
	}

	// 基本速度を計算
	int baseSpeed = maxDistance / cfg::PROCESS_SPEED_DIV;

	// 最大速度でクリップ
	baseSpeed = (baseSpeed > cfg::PROCESS_MAX_SPEED) ? cfg::PROCESS_MAX_SPEED
													 : baseSpeed;

	// 計算される角度（クリップ前）
	// ハンドリング用の最近接角度が無い場合は直進
	float steerSourceAngle =
		(minHandleDistance == INT32_MAX) ? 0.0f : minHandleAngle;

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
		float curveRatio = (float)cfg::STEER_ANGLE_MAX_DEG / absAngle;
		float curveFactor = cfg::STEER_CURVE_SPEED_FACTOR * curveRatio;
		int curveReducedSpeed = (int)(limitedSpeed * curveFactor);
		limitedSpeed = curveReducedSpeed;
		if (override_reason == "NONE") {
			override_reason = "CURVE";
		} else {
			override_reason += "+CURVE";
		}
	}

	int roundedAngle = static_cast< int >(std::round(calculatedAngle));

	std::partial_sort(
		candidates.begin(),
		candidates.begin() + std::min< size_t >(3, candidates.size()),
		candidates.end(), [](const CandidateScore &a, const CandidateScore &b) {
			return a.score > b.score;
		});

	if (telemetry_) {
		TelemetrySample sample;
		sample.ts_us = mc::core::Time::us();
		sample.run_id = run_id;
		sample.tick = tick;
		sample.scan_id = scan_id;
		sample.best_angle_deg = max;
		sample.best_dist_mm = maxDistance;
		sample.best_score = 1.0f;
		sample.min_handle_angle_deg = minHandleAngle;
		sample.min_handle_dist_mm = minHandleDistance;
		sample.path_obst_mm = minObstacleOnPath;
		sample.base_speed = baseSpeed;
		sample.limited_speed = limitedSpeed;
		sample.steer_deg = roundedAngle;
		sample.steer_clamp_deg = cfg::STEER_ANGLE_MAX_DEG;
		sample.override_kind = override_reason;
		sample.th_stop_mm = cfg::PROCESS_MIN_DIST_STOP_MM;
		sample.th_safe_mm = cfg::PROCESS_MIN_DIST_SAFE_MM;

		const size_t top_n = std::min< size_t >(3, candidates.size());
		sample.top_count = top_n;
		for (size_t i = 0; i < top_n; ++i) {
			sample.top[i] = {candidates[i].angle_deg, candidates[i].distance_mm,
							 candidates[i].score};
		}
		telemetry_->emit(sample);
	}

	return ProcResult(limitedSpeed, roundedAngle);
}
