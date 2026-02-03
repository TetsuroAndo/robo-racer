#include "Process.h"
#include "config/Config.h"
#include "mc/core/Log.hpp"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct CandidateScore {
	float angle_deg{};
	int distance_mm{};
	float score{};
};

std::string compassBar(float angle_deg) {
	const int min_deg = -90;
	const int max_deg = 90;
	const int slots = 21; // odd so center aligns with 0 deg
	std::string bar(slots, '.');
	const int center = slots / 2;
	bar[center] = '0';

	float clamped = angle_deg;
	if (clamped < min_deg)
		clamped = min_deg;
	if (clamped > max_deg)
		clamped = max_deg;

	const float ratio = (clamped - min_deg) / (float)(max_deg - min_deg);
	int pos = (int)std::round(ratio * (slots - 1));
	if (pos < 0)
		pos = 0;
	if (pos >= slots)
		pos = slots - 1;
	bar[pos] = '|';

	return "[-90" + bar + "+90]";
}

std::string colorOverride(const std::string &ovr) {
	if (ovr == "NONE")
		return "\x1b[32mNONE\x1b[0m";
	if (ovr == "STOP")
		return "\x1b[31mSTOP\x1b[0m";
	return "\x1b[33m" + ovr + "\x1b[0m";
}

} // namespace

Process::Process() {}

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
		std::ostringstream json;
		json << "{\"t_us\":" << ts_us << ",\"type\":\"NO_LIDAR\""
			 << ",\"run_id\":\"" << run_id << "\",\"tick\":" << tick
			 << ",\"scan_id\":" << scan_id
			 << ",\"mode\":\"UNKNOWN\",\"cmd\":{\"v\":0,\"steer_deg\":0}}";
		MC_LOGW("event", json.str());
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

	const uint64_t ts_us = mc::core::Time::us();
	std::ostringstream telemetry;
	telemetry << std::fixed << std::setprecision(2);
	telemetry << "{\"t_us\":" << ts_us << ",\"type\":\"telemetry\""
			  << ",\"run_id\":\"" << run_id << "\",\"tick\":" << tick
			  << ",\"scan_id\":" << scan_id << ",\"mode\":\"UNKNOWN\""
			  << ",\"best\":{\"ang_deg\":" << max
			  << ",\"dist_mm\":" << maxDistance
			  << ",\"score\":1.00,\"terms\":{\"distance\":1.00}}"
			  << ",\"min_handle\":{\"ang_deg\":" << minHandleAngle
			  << ",\"dist_mm\":" << minHandleDistance << "}"
			  << ",\"path_obst_mm\":" << minObstacleOnPath
			  << ",\"cmd\":{\"v\":" << baseSpeed
			  << ",\"v_limited\":" << limitedSpeed
			  << ",\"steer_deg\":" << roundedAngle << "}"
			  << ",\"limits\":{\"steer_clamp_deg\":" << cfg::STEER_ANGLE_MAX_DEG
			  << "}"
			  << ",\"override\":{\"kind\":\"" << override_reason << "\"}";

	telemetry << ",\"top\":[";
	const size_t top_n = std::min< size_t >(3, candidates.size());
	for (size_t i = 0; i < top_n; ++i) {
		if (i > 0)
			telemetry << ",";
		telemetry << "{\"ang_deg\":" << candidates[i].angle_deg
				  << ",\"dist_mm\":" << candidates[i].distance_mm
				  << ",\"score\":" << candidates[i].score
				  << ",\"terms\":{\"distance\":" << candidates[i].score << "}}";
	}
	telemetry << "]}";
	MC_LOGI("telemetry", telemetry.str());

	static std::string last_override;
	if (override_reason != last_override) {
		std::ostringstream event;
		event << "{\"t_us\":" << ts_us << ",\"type\":\"OVERRIDE\""
			  << ",\"run_id\":\"" << run_id << "\",\"tick\":" << tick
			  << ",\"scan_id\":" << scan_id << ",\"kind\":\"" << override_reason
			  << "\",\"path_obst_mm\":" << minObstacleOnPath
			  << ",\"th_stop_mm\":" << cfg::PROCESS_MIN_DIST_STOP_MM
			  << ",\"th_safe_mm\":" << cfg::PROCESS_MIN_DIST_SAFE_MM << "}";
		MC_LOGI("event", event.str());
		last_override = override_reason;
	}
	std::ostringstream l1;
	l1 << std::fixed << std::setprecision(2);
	l1 << compassBar(max) << "  best:" << std::showpos << max
	   << "deg  score:1.00  dist:" << (maxDistance / 1000.0f) << "m";

	std::ostringstream l2;
	l2 << "top:";
	for (size_t i = 0; i < candidates.size() && i < 3; ++i) {
		l2 << " " << std::showpos << candidates[i].angle_deg << "("
		   << candidates[i].score << ")";
	}
	l2 << "  override: " << colorOverride(override_reason);

	std::ostringstream l3;
	l3 << "path_obst=" << minObstacleOnPath
	   << "mm  min_handle=" << minHandleAngle << "deg@" << minHandleDistance
	   << "mm";

	std::ostringstream l4;
	l4 << "cmd: v=" << baseSpeed << "->" << limitedSpeed
	   << "  steer=" << roundedAngle << "deg";

	static bool ui_initialized = false;
	const size_t frame_width = 88;
	auto visible_len = [](const std::string &s) {
		size_t len = 0;
		for (size_t i = 0; i < s.size(); ++i) {
			if (s[i] == '\x1b' && i + 1 < s.size() && s[i + 1] == '[') {
				i += 2;
				while (i < s.size() && s[i] != 'm')
					++i;
				continue;
			}
			++len;
		}
		return len;
	};
	auto pad = [&](const std::string &s) {
		const size_t vis = visible_len(s);
		if (vis >= frame_width - 2)
			return s;
		return s + std::string(frame_width - 2 - vis, ' ');
	};

	const std::string title = " ROBO RACER TELEMETRY ";
	const std::string top =
		"+" + std::string((frame_width - title.size()) / 2, '-') + title +
		std::string(frame_width - title.size() -
						((frame_width - title.size()) / 2) - 2,
					'-') +
		"+";
	const std::string bot = "+" + std::string(frame_width - 2, '-') + "+";

	auto line = [&](const std::string &s) { return "|" + pad(s) + "|"; };

	if (!ui_initialized) {
		std::cout << "\x1b[?25l"; // hide cursor
		std::cout << top << "\n"
				  << line(l1.str()) << "\n"
				  << line(l2.str()) << "\n"
				  << line(l3.str()) << "\n"
				  << line(l4.str()) << "\n"
				  << bot;
		std::cout << "\x1b[6A";
		ui_initialized = true;
	} else {
		std::cout << "\x1b[6A";
		std::cout << "\x1b[2K\r" << top << "\n";
		std::cout << "\x1b[2K\r" << line(l1.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l2.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l3.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l4.str()) << "\n";
		std::cout << "\x1b[2K\r" << bot;
		std::cout << "\x1b[6A";
	}
	std::cout << std::flush;

	return ProcResult(limitedSpeed, roundedAngle);
}
