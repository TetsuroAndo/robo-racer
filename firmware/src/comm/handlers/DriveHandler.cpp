#include "../../../lib/common/Math.h"
#include "../../config/Config.h"
#include "../../log/AsyncLogger.h"
#include "../registry.h"
#include <mc/proto/Ack.hpp>

namespace {

static inline int16_t rd16(const uint8_t *p) {
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t rdu16(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

class DriveHandler : public mc::IHandler {
public:
	mc::Result onFrame(const mc::proto::FrameView &f, mc::Context &ctx,
					   uint32_t now_ms) override {
		if (f.payload_len != 8) {
			if (ctx.log) {
				ctx.log->logf(mc::LogLevel::WARN, "proto",
							  "RX DRIVE invalid len=%u",
							  (unsigned)f.payload_len);
			}
			return mc::Result::Fail(mc::Errc::Invalid, "drive len");
		}

		int16_t steer_cdeg = rd16(f.payload + 0);
		int16_t speed_mm_s = rd16(f.payload + 2);
		uint16_t ttl_ms = rdu16(f.payload + 4);
		uint16_t dist_mm = rdu16(f.payload + 6);

		steer_cdeg = (int16_t)mc::clamp< int >(
			steer_cdeg, cfg::STEER_ANGLE_MIN_CDEG, cfg::STEER_ANGLE_MAX_CDEG);
		speed_mm_s = (int16_t)mc::clamp< int >(speed_mm_s, -5000, 5000);
		if (ttl_ms < 10)
			ttl_ms = 10;
		if (ttl_ms > 2000)
			ttl_ms = 2000;

		auto *st = ctx.st;
		st->last_seq = f.seq();
		st->last_cmd_ms = now_ms;
		st->cmd_expire_ms = now_ms + ttl_ms;

		st->target_steer_cdeg = steer_cdeg;
		st->target_speed_mm_s = speed_mm_s;
		st->target_ttl_ms = ttl_ms;
		st->target_dist_mm = dist_mm;

		if (ctx.log) {
			ctx.log->logf(mc::LogLevel::DEBUG, "proto",
						  "RX DRIVE seq=%u steer=%d speed=%d ttl=%u dist=%u",
						  (unsigned)st->last_seq, (int)steer_cdeg,
						  (int)speed_mm_s, (unsigned)ttl_ms, (unsigned)dist_mm);
		}

		if (f.flags() & mc::proto::FLAG_ACK_REQ) {
			mc::proto::send_ack(ctx.tx, f.seq());
		}

		return mc::Result::Ok();
	}
};

} // namespace

MC_REGISTER_HANDLER(mc::proto::Type::DRIVE, DriveHandler)
