#include "Process.h"
#include "config/Config.h"

#include <cmath>
#include <iostream>

Process::Process() {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData) const {
	float max = 0;
	int maxDistance = -1;
	float min = 0;
	int minDistance = INT32_MAX;
	// std::cout << lidarData.size() << "\n";
	for (const auto &i : lidarData) {
		if (cfg::PROCESS_ANGLE_MIN_DEG <= i.angle &&
			i.angle <= cfg::PROCESS_ANGLE_MAX_DEG) {
			if (maxDistance < i.distance) {
				max = i.angle;
				maxDistance = i.distance;
			}
			if (i.distance < minDistance) {
				min = i.angle;
				minDistance = i.distance;
			}
		}
	}
	std::cout << "MaxDist: " << maxDistance << " at " << max
			  << " | MinDist: " << minDistance << " at " << min << "\n";

	// 基本速度を計算
	int baseSpeed = maxDistance / cfg::PROCESS_SPEED_DIV;

	// 計算される角度（クリップ前）
	float calculatedAngle = min * cfg::PROCESS_MIN_ANGLE_SIGN * cfg::PROCESS_STEER_GAIN;
	float absAngle = std::fabs(calculatedAngle);

	// 障害物距離に基づいて速度を制限
	int limitedSpeed = baseSpeed;
	if (minDistance <= cfg::PROCESS_MIN_DIST_STOP_MM) {
		// 停止距離以下 → 停止
		limitedSpeed = 0;
		std::cout << "STOP: obstacle too close (" << minDistance << "mm)"
				  << std::endl;
	} else if (minDistance < cfg::PROCESS_MIN_DIST_SAFE_MM) {
		// 安全距離未満 → 減速
		limitedSpeed = (int)(baseSpeed * cfg::PROCESS_MIN_DIST_SPEED_FACTOR);
		std::cout << "SLOW: obstacle near (" << minDistance << "mm), speed: "
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
