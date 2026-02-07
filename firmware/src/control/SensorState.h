#pragma once

#include <stdint.h>

struct Tsd20State {
	bool ready = false;
	bool valid = false;
	uint16_t mm = 0;
	uint8_t fail_count = 0;
	// センサ値の「鮮度」を表す経過時間 [ms]。
	// 0xFFFF は「不明／未取得」を意味する。
	uint16_t age_ms = 0xFFFFu;
	// 直近の測定周期 [ms]。0 の場合は未定義。
	uint16_t period_ms = 0;
};
