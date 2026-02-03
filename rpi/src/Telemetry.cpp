#include "Telemetry.h"

#include "mc/core/Log.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

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

size_t visibleLen(const std::string &s) {
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
}

} // namespace

void TelemetryEmitter::emit(const TelemetrySample &s) {
	emitJson_(s);
	emitOverrideEvent_(s);
	if (ui_enabled_)
		emitUi_(s);
}

void TelemetryEmitter::emitNoLidar(uint64_t ts_us, const std::string &run_id,
								   uint64_t tick, uint64_t scan_id) {
	std::ostringstream json;
	json << "{\"t_us\":" << ts_us << ",\"type\":\"NO_LIDAR\""
		 << ",\"run_id\":\"" << run_id << "\",\"tick\":" << tick
		 << ",\"scan_id\":" << scan_id
		 << ",\"mode\":\"UNKNOWN\",\"cmd\":{\"v\":0,\"steer_deg\":0}}";
	MC_LOGW("event", json.str());
}

void TelemetryEmitter::shutdownUi() {
	if (!ui_initialized_)
		return;
	std::cout << "\x1b[?25h" << std::flush;
}

void TelemetryEmitter::emitJson_(const TelemetrySample &s) {
	std::ostringstream telemetry;
	telemetry << std::fixed << std::setprecision(2);
	telemetry << "{\"t_us\":" << s.ts_us << ",\"type\":\"telemetry\""
			  << ",\"run_id\":\"" << s.run_id << "\",\"tick\":" << s.tick
			  << ",\"scan_id\":" << s.scan_id << ",\"mode\":\"" << s.mode
			  << "\""
			  << ",\"best\":{\"ang_deg\":" << s.best_angle_deg
			  << ",\"dist_mm\":" << s.best_dist_mm
			  << ",\"score\":" << s.best_score
			  << ",\"terms\":{\"distance\":" << s.best_score << "}}"
			  << ",\"min_handle\":{\"ang_deg\":" << s.min_handle_angle_deg
			  << ",\"dist_mm\":" << s.min_handle_dist_mm << "}"
			  << ",\"path_obst_mm\":" << s.path_obst_mm
			  << ",\"cmd\":{\"v\":" << s.base_speed
			  << ",\"v_limited\":" << s.limited_speed
			  << ",\"steer_deg\":" << s.steer_deg << "}"
			  << ",\"limits\":{\"steer_clamp_deg\":" << s.steer_clamp_deg << "}"
			  << ",\"override\":{\"kind\":\"" << s.override_kind << "\"}"
			  << ",\"top\":[";
	for (size_t i = 0; i < s.top_count; ++i) {
		if (i > 0)
			telemetry << ",";
		telemetry << "{\"ang_deg\":" << s.top[i].angle_deg
				  << ",\"dist_mm\":" << s.top[i].distance_mm
				  << ",\"score\":" << s.top[i].score
				  << ",\"terms\":{\"distance\":" << s.top[i].score << "}}";
	}
	telemetry << "]}";
	MC_LOGI("telemetry", telemetry.str());
}

void TelemetryEmitter::emitOverrideEvent_(const TelemetrySample &s) {
	if (s.override_kind == last_override_)
		return;
	std::ostringstream event;
	event << "{\"t_us\":" << s.ts_us << ",\"type\":\"OVERRIDE\""
		  << ",\"run_id\":\"" << s.run_id << "\",\"tick\":" << s.tick
		  << ",\"scan_id\":" << s.scan_id << ",\"kind\":\"" << s.override_kind
		  << "\",\"path_obst_mm\":" << s.path_obst_mm
		  << ",\"th_stop_mm\":" << s.th_stop_mm
		  << ",\"th_safe_mm\":" << s.th_safe_mm << "}";
	MC_LOGI("event", event.str());
	last_override_ = s.override_kind;
}

void TelemetryEmitter::emitUi_(const TelemetrySample &s) {
	std::ostringstream l1;
	l1 << std::fixed << std::setprecision(2);
	l1 << compassBar(s.best_angle_deg) << "  best:" << std::showpos
	   << s.best_angle_deg << "deg  score:" << s.best_score
	   << "  dist:" << (s.best_dist_mm / 1000.0f) << "m";

	std::ostringstream l2;
	l2 << "top:";
	for (size_t i = 0; i < s.top_count; ++i) {
		l2 << " " << std::showpos << s.top[i].angle_deg << "(" << s.top[i].score
		   << ")";
	}
	l2 << "  override: " << colorOverride(s.override_kind);

	std::ostringstream l3;
	l3 << "path_obst=" << s.path_obst_mm
	   << "mm  min_handle=" << s.min_handle_angle_deg << "deg@"
	   << s.min_handle_dist_mm << "mm";

	std::ostringstream l4;
	l4 << "cmd: v=" << s.base_speed << "->" << s.limited_speed
	   << "  steer=" << s.steer_deg << "deg";

	const size_t frame_width = 88;
	auto pad = [&](const std::string &sline) {
		const size_t vis = visibleLen(sline);
		if (vis >= frame_width - 2)
			return sline;
		return sline + std::string(frame_width - 2 - vis, ' ');
	};

	const std::string title = " ROBO RACER TELEMETRY ";
	const std::string top =
		"+" + std::string((frame_width - title.size()) / 2, '-') + title +
		std::string(frame_width - title.size() -
						((frame_width - title.size()) / 2) - 2,
					'-') +
		"+";
	const std::string bot = "+" + std::string(frame_width - 2, '-') + "+";

	auto line = [&](const std::string &sline) {
		return "|" + pad(sline) + "|";
	};

	if (!ui_initialized_) {
		std::cout << "\x1b[?25l"; // hide cursor
		std::cout << top << "\n"
				  << line(l1.str()) << "\n"
				  << line(l2.str()) << "\n"
				  << line(l3.str()) << "\n"
				  << line(l4.str()) << "\n"
				  << bot;
		std::cout << "\x1b[6A";
		ui_initialized_ = true;
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
}
