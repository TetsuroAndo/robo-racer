#include "Process.h"
#include "config/Config.h"

#include <cmath>
#include <iostream>

Process::Process() {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData, float lastSteerAngle) const {
	float max = 0;
	int maxDistance = -1;
	float minHandleAngle = 0;
	int minHandleDistance = INT32_MAX;
	int minObstacleOnPath = INT32_MAX;
	
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
	std::cout << "MaxDist: " << maxDistance << " at " << max
			  << " | MinHandleDist: " << minHandleDistance << " at "
			  << minHandleAngle << " | PathObstacle: " << minObstacleOnPath
			  << " (checkRange: " << pathCheckMin << "~" << pathCheckMax << "°)\n";

	// データが無い場合は安全側で停止
	if (maxDistance < 0) {
		std::cout << "WARN: no lidar points in handling window" << std::endl;
		return ProcResult(0, 0);
	}

	// 基本速度を計算
	int baseSpeed = maxDistance / cfg::PROCESS_SPEED_DIV;

	// 最大速度でクリップ
	baseSpeed = (baseSpeed > cfg::PROCESS_MAX_SPEED) ? cfg::PROCESS_MAX_SPEED : baseSpeed;

	// 計算される角度（クリップ前）
	// ハンドリング用の最近接角度が無い場合は直進
	float steerSourceAngle = (minHandleDistance == INT32_MAX) ? 0.0f : minHandleAngle;
	
	// 現在のステアリング方向との角度差に基づいた重み付け
	// 角度差が小さいほど、現在の方向をより優先
	if (minHandleDistance != INT32_MAX) {
		float angleDiff = std::abs(steerSourceAngle - clampedSteerAngle);
		// 角度差に基づいた重み（0°:1.0、90°:減少）
		float directionWeight = 1.0f - (angleDiff / 90.0f) * cfg::PROCESS_DIRECTION_WEIGHT;
		// 現在の方向との平均を取ることで滑らかに遷移
		float blendFactor = cfg::PROCESS_DIRECTION_WEIGHT;
		steerSourceAngle = steerSourceAngle * (1.0f - blendFactor) + clampedSteerAngle * blendFactor * directionWeight;
	}
	
	float calculatedAngle = steerSourceAngle * cfg::PROCESS_MIN_ANGLE_SIGN * cfg::PROCESS_STEER_GAIN;
	float absAngle = std::fabs(calculatedAngle);

	// 進行経路上の障害物距離に基づいて速度を制限
	int limitedSpeed = baseSpeed;
	if (minObstacleOnPath <= cfg::PROCESS_MIN_DIST_STOP_MM) {
		// 停止距離以下 → 停止
		limitedSpeed = 0;
		std::cout << "STOP: obstacle on path too close (" << minObstacleOnPath << "mm)"
				  << std::endl;
	} else if (minObstacleOnPath < cfg::PROCESS_MIN_DIST_SAFE_MM) {
		// 安全距離未満 → 減速
		limitedSpeed = (int)(baseSpeed * cfg::PROCESS_MIN_DIST_SPEED_FACTOR);
		std::cout << "SLOW: obstacle on path near (" << minObstacleOnPath << "mm), speed: "
				  << baseSpeed << " → " << limitedSpeed << std::endl;
	}

	// 急カーブによる速度制限
	// 計算角度が物理上限を超える場合、その比率に応じて速度を減速
	if (absAngle > cfg::STEER_ANGLE_MAX_DEG) {
		float curveRatio = (float)cfg::STEER_ANGLE_MAX_DEG / absAngle;
		float curveFactor = cfg::STEER_CURVE_SPEED_FACTOR + (1.0f - cfg::STEER_CURVE_SPEED_FACTOR) * curveRatio;
		int curveReducedSpeed = (int)(limitedSpeed * curveFactor);
		std::cout << "CURVE: sharp turn needed (" << absAngle << "°), ratio: "
				  << curveRatio << ", speed: " << limitedSpeed << " → "
				  << curveReducedSpeed << std::endl;
		limitedSpeed = curveReducedSpeed;
	}

	return ProcResult(limitedSpeed, calculatedAngle);
}
