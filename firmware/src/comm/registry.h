#pragma once
#include <stdint.h>
#include "mc_proto.h"
#include "../../lib/common/Result.h"

namespace mc {

enum class Mode : uint8_t { MANUAL = 0, AUTO = 1 };

struct ControlState {
	volatile Mode mode = Mode::MANUAL;
	volatile bool killed = false;

	volatile uint16_t last_seq = 0;
	volatile uint32_t last_cmd_ms = 0;
	volatile uint32_t cmd_expire_ms = 0;

	volatile int16_t target_steer_cdeg = 0;
	volatile int16_t target_speed_mm_s = 0;
	volatile uint16_t target_ttl_ms = 100;
	volatile uint16_t target_dist_mm = 0;
};

struct Context {
	ControlState* st;
	HardwareSerial* uart;
};

class IHandler {
public:
	virtual ~IHandler() {}
	virtual mc::Result onFrame(const proto::FrameView& f, Context& ctx, uint32_t now_ms) = 0;
};

class Registry {
public:
	static Registry& instance();

	bool add(uint8_t type, IHandler* h);
	IHandler* get(uint8_t type);

private:
	Registry();
	static constexpr int MAX = 16;
	struct Entry { uint8_t type; IHandler* h; };
	Entry _e[MAX];
	int _n;
};

class AutoRegister {
public:
	AutoRegister(uint8_t type, IHandler* h) { Registry::instance().add(type, h); }
};

} // namespace mc

#define MC_REGISTER_HANDLER(TYPE_ENUM, CLASSNAME) \
	static CLASSNAME g_##CLASSNAME; \
	static mc::AutoRegister g_reg_##CLASSNAME((uint8_t)(TYPE_ENUM), &g_##CLASSNAME);
