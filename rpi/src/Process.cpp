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

static constexpr float kDegToRad = 0.01745329252f;

// bins(1degごとの最短距離)から、舵角に応じた「円弧コリドー上の最短衝突距離」を作る
static int
steerAwareClearanceMm(const std::array< int32_t, cfg::FTG_BIN_COUNT > &bins,
					  float steer_deg, float half_w_mm) {
	const float steer = steer_deg * cfg::FTG_STEER_MODEL_SIGN;
	const float abs_deg = std::fabs(steer);
	int best = cfg::FTG_ARC_CLEARANCE_MAX_MM;

	// ほぼ直進：矩形コリドー（|y|<=half_w, x>0）の最短x
	if (abs_deg < cfg::FTG_ARC_STRAIGHT_DEG) {
		for (int angle = cfg::FTG_ANGLE_MIN_DEG;
			 angle <= cfg::FTG_ANGLE_MAX_DEG; ++angle) {
			const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
			const int32_t r_i = bins[(size_t)idx];
			if (r_i <= 0)
				continue;
			const float th = (float)angle * kDegToRad;
			const float r = (float)r_i;
			const float x = r * std::cos(th);
			const float y = r * std::sin(th);
			if (x <= 0.0f)
				continue;
			if (std::fabs(y) <= half_w_mm) {
				const int x_i = (int)std::lround(x);
				if (x_i < best)
					best = x_i;
			}
		}
		return best;
	}

	// 円弧：R = L / tan(|δ|)
	const float L_mm = cfg::FTG_WHEELBASE_M * 1000.0f;
	const float delta = abs_deg * kDegToRad;
	const float tan_d = std::tan(delta);
	if (tan_d <= 1e-4f)
		return best;
	const float R = std::max(1.0f, L_mm / tan_d);

	// 右旋回を左旋回フレームへミラー（yだけ符号反転）
	const float sign = (steer >= 0.0f) ? 1.0f : -1.0f;

	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
		 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const int32_t r_i = bins[(size_t)idx];
		if (r_i <= 0)
			continue;
		const float th = (float)angle * kDegToRad;
		const float r = (float)r_i;
		const float x = r * std::cos(th);
		const float y = r * std::sin(th);
		if (x <= 0.0f)
			continue;

		const float yL = y * sign;
		// 円中心(0,R)からの距離 rho が [R-half_w, R+half_w]
		// に入れば"掃引幅にいる"
		const float rho = std::hypot(x, yL - R);
		if (std::fabs(rho - R) > half_w_mm)
			continue;

		// 原点(0,0)から円弧に沿った距離 s = t*R,  t = atan2(x, R - yL)
		const float t = std::atan2(x, (R - yL));
		if (t < 0.0f)
			continue;
		const int s_i = (int)std::lround(t * R);
		// 車体現在位置の側壁（arc距離が極小）を無視する。
		// 旋回の反対側にある壁がコリドー幅に入るが、車は離れる方向なので安全。
		if (s_i < cfg::FTG_ARC_MIN_AHEAD_MM)
			continue;
		if (s_i < best)
			best = s_i;
	}
	return best;
}
} // namespace

Process::Process(TelemetryEmitter *telemetry) : telemetry_(telemetry) {}

Process::~Process() {}

ProcResult Process::proc(const std::vector< LidarData > &lidarData,
						 float lastSteerAngle, uint64_t tick, uint64_t scan_id,
						 const std::string &run_id,
						 const MotionState *motion) const {
	const uint64_t t0_us = mc::core::Time::us();
	static constexpr float kRadToDeg = 57.2957795f;
	const float max_steer = (float)mc_config::STEER_ANGLE_MAX_DEG;
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
	float pred_margin_mm = 0.0f;
	float a_brake_use = 0.0f;
	bool has_brake_cap = false;
	if (cfg::FTG_PREDICT_ENABLE && motion && motion->valid &&
		motion->calibrated && motion->age_ms <= cfg::FTG_IMU_MAX_AGE_MS) {
		const float v_est = std::max(0.0f, (float)motion->v_est_mm_s);
		const float a_long = (float)motion->a_long_mm_s2;
		const float a_pos = std::max(
			0.0f, std::min(a_long, (float)cfg::FTG_PREDICT_ACCEL_MAX_MM_S2));
		float a_brake = (float)motion->a_brake_cap_mm_s2;
		if (a_brake <= 0.0f)
			a_brake = (float)cfg::FTG_PREDICT_BRAKE_MM_S2;
		if (a_brake < (float)cfg::FTG_PREDICT_BRAKE_MIN_MM_S2)
			a_brake = (float)cfg::FTG_PREDICT_BRAKE_MIN_MM_S2;
		else if (a_brake > (float)cfg::FTG_PREDICT_BRAKE_MAX_MM_S2)
			a_brake = (float)cfg::FTG_PREDICT_BRAKE_MAX_MM_S2;
		a_brake_use = a_brake;
		has_brake_cap = true;
		const float tau = (float)cfg::FTG_PREDICT_LATENCY_MS / 1000.0f;
		const float d_react = v_est * tau + 0.5f * a_pos * tau * tau;
		pred_margin_mm = d_react;
		if (pred_margin_mm > (float)cfg::FTG_PREDICT_MARGIN_MAX_MM)
			pred_margin_mm = (float)cfg::FTG_PREDICT_MARGIN_MAX_MM;
	}
	const int pred_margin_i =
		(pred_margin_mm > 0.0f) ? (int)std::lround(pred_margin_mm) : 0;
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
		int dist = p.distance;
		if (dist <= 0)
			continue;
		if (pred_margin_i > 0) {
			dist -= pred_margin_i;
			if (dist <= 0)
				continue;
		}
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		if (bins[(size_t)idx] == 0 || dist < bins[(size_t)idx])
			bins[(size_t)idx] = dist;
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
		int min_mm = std::numeric_limits< int >::max();
		for (int a = angle - n; a <= angle + n; ++a) {
			if (a < cfg::FTG_ANGLE_MIN_DEG || a > cfg::FTG_ANGLE_MAX_DEG)
				continue;
			const int a_idx = a - cfg::FTG_ANGLE_MIN_DEG;
			const int32_t raw = bins[(size_t)a_idx];
			if (raw > 0 && raw < min_mm)
				min_mm = raw;
		}
		if (min_mm != std::numeric_limits< int >::max())
			corridor_min[(size_t)idx] = min_mm;
	}

	int best_angle = 0;
	int best_dist = 0;
	float best_score = -std::numeric_limits< float >::infinity();
	float selected_target = 0.0f;
	bool found_gap = false;
	bool has_data = false;
	int min_corridor = std::numeric_limits< int32_t >::max();
	int min_angle = 0;

	// delta_relax: 近距離ほど舵の急変を許す（jerk_weight と同様）
	auto delta_relax = [](int depth_mm) -> float {
		if (cfg::FTG_JERK_RELAX_MM <= cfg::FTG_NEAR_OBSTACLE_MM)
			return 1.0f;
		if (depth_mm >= cfg::FTG_JERK_RELAX_MM)
			return 1.0f;
		float s = (float)(depth_mm - cfg::FTG_NEAR_OBSTACLE_MM) /
				  (float)(cfg::FTG_JERK_RELAX_MM - cfg::FTG_NEAR_OBSTACLE_MM);
		if (s < 0.0f)
			s = 0.0f;
		else if (s > 1.0f)
			s = 1.0f;
		return s * s;
	};

	// gap スコアリングヘルパー関数
	auto scoreGap = [&](int gap_start, int gap_end) -> void {
		const int width = gap_end - gap_start + 1;
		if (width < cfg::FTG_GAP_MIN_WIDTH_DEG)
			return;
		std::vector< int > ds;
		ds.reserve((size_t)width);
		for (int a = gap_start; a <= gap_end; ++a) {
			const int a_idx = a - cfg::FTG_ANGLE_MIN_DEG;
			const int v = corridor_min[(size_t)a_idx];
			if (v > 0)
				ds.push_back(v);
		}
		if (ds.empty())
			return;
		std::sort(ds.begin(), ds.end());
		const size_t q_idx =
			(ds.size() > 1)
				? (size_t)((float)(ds.size() - 1) * cfg::FTG_GAP_DEPTH_Q)
				: 0;
		const int depth_mm = ds[std::min(q_idx, ds.size() - 1)];
		float w_sum = 0.0f;
		float angle_sum = 0.0f;
		int peak_angle = gap_start;
		int peak_dist = 0;
		for (int a = gap_start; a <= gap_end; ++a) {
			const int a_idx = a - cfg::FTG_ANGLE_MIN_DEG;
			const int d = corridor_min[(size_t)a_idx];
			if (d <= 0)
				continue;
			if (d > peak_dist) {
				peak_dist = d;
				peak_angle = a;
			}
			const float excess =
				std::max(0.0f, (float)(d - cfg::FTG_NEAR_OBSTACLE_MM));
			const float w = std::pow(excess, cfg::FTG_GAP_WEIGHT_GAMMA);
			w_sum += w;
			angle_sum += w * (float)a;
		}
		const float target =
			(w_sum > 0.0f) ? (angle_sum / w_sum) : (float)peak_angle;
		const float depth_n =
			std::min(1.0f, (float)depth_mm / (float)cfg::FTG_GAP_DEPTH_SAT_MM);
		const float width_n =
			std::min(1.0f, (float)width / (float)cfg::FTG_GAP_WIDTH_REF_DEG);
		const float base =
			depth_n * (1.0f + cfg::FTG_GAP_WIDTH_WEIGHT * width_n);
		const float a_norm =
			std::fabs(target) / (float)mc_config::STEER_ANGLE_MAX_DEG;
		const float d_norm = std::fabs(target - clamped_last) /
							 (float)mc_config::STEER_ANGLE_MAX_DEG;
		const float dr = delta_relax(depth_mm);
		const float pen = cfg::FTG_GAP_TURN_PENALTY * a_norm +
						  cfg::FTG_GAP_DELTA_PENALTY * dr * (d_norm * d_norm);
		const float score = base - pen;
		if (score > best_score) {
			best_score = score;
			selected_target = target;
			best_angle = peak_angle;
			best_dist = depth_mm;
			found_gap = true;
		}
	};

	// gap 抽出・スコアリング
	bool in_gap = false;
	int gap_s = 0;
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
		const bool free = (d_mm >= cfg::FTG_GAP_FREE_MM);
		if (free) {
			if (!in_gap) {
				in_gap = true;
				gap_s = angle;
			}
		} else {
			if (in_gap) {
				const int gap_e = angle - 1;
				scoreGap(gap_s, gap_e);
				in_gap = false;
			}
		}
	}
	if (in_gap) {
		scoreGap(gap_s, cfg::FTG_ANGLE_MAX_DEG);
	}

	if (!has_data) {
		const uint64_t ts_us = mc::core::Time::us();
		if (telemetry_)
			telemetry_->emitNoLidar(ts_us, run_id, tick, scan_id);
		return ProcResult(0, 0);
	}

	// gapが無い=即停止 はやめる（誤爆源）。舵は0へ戻さず進行方向を維持し、
	// 停止判定は steer-aware clearance に任せる。
	float target_angle_f = found_gap ? selected_target : clamped_last;
	// Safety: ensure target stays within steering limits even transiently.
	target_angle_f = std::max(-max_steer, std::min(max_steer, target_angle_f));

	const float max_delta = cfg::FTG_STEER_SLEW_DEG_PER_S * dt_s;
	float applied_angle_f = target_angle_f;
	const float delta = applied_angle_f - clamped_last;
	if (delta > max_delta)
		applied_angle_f = clamped_last + max_delta;
	else if (delta < -max_delta)
		applied_angle_f = clamped_last - max_delta;

	int out_angle = static_cast< int >(std::lround(applied_angle_f));

	int corridor_min_mm = 0;
	{
		const int out_idx = out_angle - cfg::FTG_ANGLE_MIN_DEG;
		if (!found_gap) {
			// 進行方向（舵維持）のコリドーを使用。スタックを避けるため舵は維持済み
			if (out_idx >= 0 && out_idx < cfg::FTG_BIN_COUNT &&
				corridor_min[(size_t)out_idx] > 0) {
				corridor_min_mm = corridor_min[(size_t)out_idx];
			} else {
				corridor_min_mm =
					(min_corridor == std::numeric_limits< int32_t >::max())
						? 0
						: min_corridor;
			}
			best_angle = static_cast< int >(std::lround(clamped_last));
			best_dist = corridor_min_mm;
		} else {
			if (out_idx >= 0 && out_idx < cfg::FTG_BIN_COUNT) {
				corridor_min_mm = corridor_min[(size_t)out_idx];
			}
			if (corridor_min_mm <= 0) {
				out_angle = best_angle;
				applied_angle_f = (float)best_angle;
				corridor_min_mm = best_dist;
			}
		}
	}
	// Single steer clamp (replaces redundant clamps at target and out_angle)
	out_angle = static_cast< int >(
		std::max(-mc_config::STEER_ANGLE_MAX_DEG,
				 std::min(mc_config::STEER_ANGLE_MAX_DEG, (float)out_angle)));
	applied_angle_f = (float)out_angle;

	// steer-aware path clearance（「真横の壁」を無視できる）
	// 現在舵と指令舵を分離保持。停止判定は指令舵側のみで行い、
	// 指令舵に十分なクリアランスがあれば即停止せず低速復帰可能にする。
	int path_clearance_mm = corridor_min_mm; // fallback: 旧直線コリドー
	int c_clearance_now = 0;
	int c_clearance_cmd = 0;
	if (cfg::FTG_ARC_CLEARANCE_ENABLE) {
		const float half_w_mm =
			(cfg::FTG_CAR_WIDTH_M * 0.5f + cfg::FTG_MARGIN_M) * 1000.0f;
		c_clearance_now = steerAwareClearanceMm(bins, clamped_last, half_w_mm);
		c_clearance_cmd =
			steerAwareClearanceMm(bins, applied_angle_f, half_w_mm);
		// 指令舵側のクリアランスで判定（復帰可能方向が十分なら停止しない）
		path_clearance_mm = c_clearance_cmd;
		if (path_clearance_mm <= 0)
			path_clearance_mm = c_clearance_now;
		if (path_clearance_mm <= 0)
			path_clearance_mm = corridor_min_mm;
	}

	const bool blocked = (path_clearance_mm > 0) &&
						 (path_clearance_mm <= cfg::FTG_NEAR_OBSTACLE_MM);
	const bool warn = (path_clearance_mm > 0) &&
					  (path_clearance_mm < cfg::FTG_WARN_OBSTACLE_MM);

	if (blocked) {
		// STOP時でも舵は維持（次の再開で刺さりにくくする）
		// out_speed は後段で blocked により 0 のまま
	}

	int out_speed = 0;
	if (!blocked && path_clearance_mm > 0) {
		int d_speed_mm = path_clearance_mm;
		const int out_idx = out_angle - cfg::FTG_ANGLE_MIN_DEG;
		if (out_idx >= 0 && out_idx < cfg::FTG_BIN_COUNT) {
			const int smoothed_mm =
				static_cast< int >(std::lround(smoothed[(size_t)out_idx]));
			if (smoothed_mm > 0) {
				if (d_speed_mm > 0)
					d_speed_mm = std::min(d_speed_mm, smoothed_mm);
				else
					d_speed_mm = smoothed_mm;
			}
		}

		const float v_min = (float)cfg::FTG_SPEED_MIN_MM_S;
		const float v_max = (float)cfg::FTG_SPEED_MAX_MM_S;
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
							   (float)mc_config::STEER_ANGLE_MAX_DEG);
		static constexpr float kPiOver2 = 1.57079632679f;
		const float v_steer =
			v_min + (v_max - v_min) * std::cos(steer_ratio * kPiOver2);

		const float v_final = std::min(v_dist, v_steer);
		const int speed =
			(int)std::lround(std::max(v_min, std::min(v_max, v_final)));
		out_speed = speed;

		// turn-cap: 舵が追いつくまでの時間に対して、距離から速度上限を作る
		const float slew = std::max(1.0f, cfg::FTG_STEER_SLEW_DEG_PER_S);
		const float t_turn = std::fabs((float)out_angle - clamped_last) / slew;
		const float avail_mm =
			(float)std::max(0, path_clearance_mm - cfg::FTG_NEAR_OBSTACLE_MM);
		if (avail_mm > 0.0f && t_turn > 0.0f) {
			const float v_turn =
				avail_mm / (t_turn + cfg::FTG_TURN_CAP_LATENCY_S);
			const int v_cap = (int)std::lround(std::max(
				0.0f, std::min((float)cfg::FTG_SPEED_MAX_MM_S, v_turn)));
			if (v_cap < out_speed)
				out_speed = v_cap;
		}

		if (warn) {
			out_speed = std::min(out_speed, cfg::FTG_SPEED_WARN_CAP_MM_S);
		}
		if (!found_gap) {
			// gapが取れてない時は慎重に（止まらず"じわ"前進）
			out_speed = std::min(out_speed, cfg::FTG_NO_GAP_SPEED_CAP_MM_S);
		}
	}
	if (!blocked && out_speed > 0 && has_brake_cap && path_clearance_mm > 0) {
		int d_cap_mm = path_clearance_mm - cfg::FTG_NEAR_OBSTACLE_MM;
		if (d_cap_mm < 0)
			d_cap_mm = 0;
		float v_cap_mm_s = (a_brake_use > 0.0f)
							   ? std::sqrt(2.0f * a_brake_use * (float)d_cap_mm)
							   : 0.0f;
		if (v_cap_mm_s < 0.0f)
			v_cap_mm_s = 0.0f;
		if (v_cap_mm_s > (float)mc_config::SPEED_MAX_MM_S)
			v_cap_mm_s = (float)mc_config::SPEED_MAX_MM_S;
		const int v_cap = (int)std::lround(v_cap_mm_s);
		if (v_cap < out_speed)
			out_speed = v_cap;
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
			const float tele_score = std::max(
				0.0f,
				std::min(1.0f, (float)d_mm / (float)cfg::FTG_GAP_DEPTH_SAT_MM));
			if (tele_score > max_score)
				max_score = tele_score;
			candidates.push_back(
				CandidateScore{(float)angle, d_mm, tele_score});

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
		if (path_clearance_mm <= cfg::FTG_NEAR_OBSTACLE_MM) {
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
		sample.best_score = std::max(
			0.0f, std::min(1.0f, (float)best_dist /
									 (float)cfg::FTG_GAP_DEPTH_SAT_MM));
		sample.best_delta_deg = best_delta;
		sample.min_handle_angle_deg = (float)min_angle;
		sample.min_handle_dist_mm =
			(min_corridor == std::numeric_limits< int32_t >::max())
				? 0
				: (int)min_corridor;
		sample.path_obst_mm =
			path_clearance_mm; // 実際の進行（円弧）に沿った距離
		sample.front_dist_mm = corridor_min_mm; // 旧：直線コリドー（比較用）
		sample.side_dist_mm = std::nullopt;
		sample.base_speed = out_speed;
		sample.limited_speed = out_speed;
		sample.steer_deg = out_angle;
		sample.raw_steer_deg = target_angle_f;
		sample.curve_ratio = 1.0f;
		sample.speed_factor = (out_speed > 0) ? 1.0f : 0.0f;
		sample.steer_clamp_deg = mc_config::STEER_ANGLE_MAX_DEG;
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
