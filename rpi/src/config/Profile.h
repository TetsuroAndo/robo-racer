#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

namespace cfg {

// 0..3 : 守り -> 攻め
enum class Profile : uint8_t {
	Safe = 0,
	Mid = 1, // 現状相当（デフォルト）
	Fast = 2,
	Attack = 3,
};

struct ProfileParams {
	const char *name;

	// 速度系（上限や、危険域での上限）
	float v_max_scale;        // cfg::FTG_SPEED_MAX_MM_S に乗算
	float warn_cap_scale;     // cfg::FTG_SPEED_WARN_CAP_MM_S に乗算
	float no_gap_cap_scale;   // cfg::FTG_NO_GAP_SPEED_CAP_MM_S に乗算
	int creep_speed_mm_s;     // 最低速度

	// 障害物しきい値（Stop / Warn）
	int near_obstacle_mm;     // 近すぎ（実質 stop 域）
	int warn_obstacle_mm;     // 減速域

	// ステア・追従（速度低下の床、ステア追従の速さ）
	float steer_speed_floor;  // 0.0=従来、>0 で曲がっても速度を落としすぎない
	float steer_slew_scale;   // cfg::FTG_STEER_SLEW_DEG_PER_S に乗算

	// Gap スコアのペナルティ（攻めほど小さくして "刺さる角" を許容）
	float gap_turn_penalty_scale;
	float gap_delta_penalty_scale;

	// 予測マージン（守りほど大きい）
	float pred_margin_scale;
};

inline Profile clampProfileInt(int v) {
	if (v < 0)
		v = 0;
	if (v > 3)
		v = 3;
	return static_cast< Profile >(v);
}

inline const ProfileParams &profileParams(Profile p) {
	// NOTE: profile 1(Mid) は現状の cfg 値に一致するようにしている
	static const ProfileParams tbl[] = {
		// 0 SAFE
		{"SAFE", 0.42f, 0.85f, 0.85f, 180, 130, 260, 0.00f, 0.85f, 1.20f,
		 1.30f, 1.20f},
		// 1 MID (current)
		{"MID", 1.00f, 1.00f, 1.00f, 220, 100, 200, 0.00f, 1.00f, 1.00f,
		 1.00f, 1.00f},
		// 2 FAST
		{"FAST", 1.05f, 1.10f, 1.10f, 250, 90, 180, 0.20f, 1.10f, 0.80f,
		 0.80f, 0.95f},
		// 3 ATTACK
		{"ATTACK", 1.10f, 1.20f, 1.20f, 280, 80, 160, 0.30f, 1.20f, 0.60f,
		 0.60f, 0.90f},
	};
	return tbl[static_cast< uint8_t >(p)];
}

inline const char *profileName(Profile p) { return profileParams(p).name; }

inline bool parseProfileToken(const std::string &s, Profile &out) {
	// 数字 0..3 のみサポート（必要なら "safe/mid/fast/attack" も足せる）
	char *end = nullptr;
	long v = std::strtol(s.c_str(), &end, 10);
	if (!end || *end != '\0')
		return false;
	if (v < 0 || v > 3)
		return false;
	out = static_cast< Profile >(v);
	return true;
}

} // namespace cfg
