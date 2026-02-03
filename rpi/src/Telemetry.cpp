#include "Telemetry.h"

#include "mc/core/Log.hpp"
#include "mc/core/Time.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
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
	refreshMetrics_();
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

void TelemetryEmitter::updateStatus(uint8_t auto_active, uint16_t faults,
									int16_t speed_mm_s, int16_t steer_cdeg,
									uint16_t age_ms) {
	status_.valid = true;
	status_.auto_active = auto_active;
	status_.faults = faults;
	status_.speed_mm_s = speed_mm_s;
	status_.steer_cdeg = steer_cdeg;
	status_.age_ms = age_ms;
	status_.ts_us = mc::core::Time::us();
}

void TelemetryEmitter::refreshMetrics_() {
	if (metrics_log_path_.empty())
		return;
	const uint64_t now_us = mc::core::Time::us();
	if (now_us - metrics_last_read_us_ < 1000 * 1000)
		return;
	metrics_last_read_us_ = now_us;

	std::ifstream ifs(metrics_log_path_, std::ios::binary);
	if (!ifs.is_open())
		return;
	ifs.seekg(0, std::ios::end);
	const std::streamoff end_off = ifs.tellg();
	const std::streamoff back = 1024;
	std::streamoff start = end_off - back;
	if (start < 0)
		start = 0;
	ifs.seekg(start);
	std::string buf((size_t)(end_off - start), '\0');
	ifs.read(&buf[0], buf.size());

	size_t last_nl = buf.find_last_of('\n');
	if (last_nl == std::string::npos)
		return;
	size_t prev_nl = buf.find_last_of('\n', last_nl - 1);
	size_t line_start = (prev_nl == std::string::npos) ? 0 : prev_nl + 1;
	std::string line = buf.substr(line_start, last_nl - line_start);

	auto find_num = [&](const char *key, uint64_t &val) -> bool {
		const std::string k = std::string(key);
		const size_t pos = line.find(k);
		if (pos == std::string::npos)
			return false;
		size_t i = pos + k.size();
		size_t end = i;
		while (end < line.size() && std::isdigit(line[end]))
			++end;
		if (end == i)
			return false;
		val = std::stoull(line.substr(i, end - i));
		return true;
	};

	uint64_t temp = 0, cpu = 0, mem_used = 0, mem_total = 0;
	const bool ok = find_num("temp_cdeg=", temp) &&
					find_num("cpu_permille=", cpu) &&
					find_num("mem_kb=", mem_used);
	if (!ok)
		return;
	const size_t slash = line.find('/', line.find("mem_kb="));
	if (slash == std::string::npos)
		return;
	size_t end_pos = slash + 1;
	while (end_pos < line.size() && std::isdigit(line[end_pos]))
		++end_pos;
	if (end_pos == slash + 1)
		return;
	mem_total = std::stoull(line.substr(slash + 1, end_pos - slash - 1));

	metrics_.valid = true;
	metrics_.cpu_temp_cdeg = (uint16_t)temp;
	metrics_.cpu_usage_permille = (uint16_t)cpu;
	metrics_.mem_used_kb = (uint32_t)mem_used;
	metrics_.mem_total_kb = (uint32_t)mem_total;
}

void TelemetryEmitter::emitJson_(const TelemetrySample &s) {
	std::ostringstream telemetry;
	telemetry << std::fixed << std::setprecision(2);
	telemetry
		<< "{\"t_us\":" << s.ts_us << ",\"type\":\"telemetry\""
		<< ",\"run_id\":\"" << s.run_id << "\",\"tick\":" << s.tick
		<< ",\"scan_id\":" << s.scan_id << ",\"mode\":\"" << s.mode
		<< "\",\"map_state\":\"" << s.map_state << "\""
		<< ",\"best\":{\"ang_deg\":" << s.best_angle_deg
		<< ",\"dist_mm\":" << s.best_dist_mm << ",\"score\":" << s.best_score
		<< ",\"delta_deg\":"
		<< (s.best_delta_deg ? std::to_string(*s.best_delta_deg) : "null")
		<< ",\"terms\":{\"distance\":" << s.best_score << ",\"obstacle\":"
		<< (s.score_obstacle ? std::to_string(*s.score_obstacle) : "null")
		<< ",\"width\":"
		<< (s.score_width ? std::to_string(*s.score_width) : "null")
		<< ",\"goal\":"
		<< (s.score_goal ? std::to_string(*s.score_goal) : "null")
		<< ",\"curvature\":"
		<< (s.score_curvature ? std::to_string(*s.score_curvature) : "null")
		<< ",\"stability\":"
		<< (s.score_stability ? std::to_string(*s.score_stability) : "null")
		<< ",\"map_conf\":"
		<< (s.score_map_conf ? std::to_string(*s.score_map_conf) : "null")
		<< "}}"
		<< ",\"min_handle\":{\"ang_deg\":" << s.min_handle_angle_deg
		<< ",\"dist_mm\":" << s.min_handle_dist_mm << "}"
		<< ",\"path_obst_mm\":" << s.path_obst_mm << ",\"front_dist_mm\":"
		<< (s.front_dist_mm ? std::to_string(*s.front_dist_mm) : "null")
		<< ",\"side_dist_mm\":"
		<< (s.side_dist_mm ? std::to_string(*s.side_dist_mm) : "null")
		<< ",\"cmd\":{\"v\":" << s.base_speed
		<< ",\"v_limited\":" << s.limited_speed
		<< ",\"steer_deg\":" << s.steer_deg
		<< ",\"raw_steer_deg\":" << s.raw_steer_deg
		<< ",\"curve_ratio\":" << s.curve_ratio
		<< ",\"speed_factor\":" << s.speed_factor << "}"
		<< ",\"limits\":{\"steer_clamp_deg\":" << s.steer_clamp_deg << "}"
		<< ",\"override\":{\"kind\":\"" << s.override_kind << "\",\"detail\":\""
		<< s.override_detail << "\"}"
		<< ",\"scan_age_ms\":"
		<< (s.scan_age_ms ? std::to_string(*s.scan_age_ms) : "null")
		<< ",\"latency_ms\":{\"planner\":"
		<< (s.planner_latency_ms ? std::to_string(*s.planner_latency_ms)
								 : "null")
		<< ",\"control\":"
		<< (s.control_latency_ms ? std::to_string(*s.control_latency_ms)
								 : "null")
		<< "}"
		<< ",\"ttl_ms\":" << (s.ttl_ms ? std::to_string(*s.ttl_ms) : "null")
		<< ",\"top\":[";
	for (size_t i = 0; i < s.top_count; ++i) {
		if (i > 0)
			telemetry << ",";
		telemetry << "{\"ang_deg\":" << s.top[i].angle_deg
				  << ",\"dist_mm\":" << s.top[i].distance_mm
				  << ",\"score\":" << s.top[i].score
				  << ",\"terms\":{\"distance\":" << s.top[i].score << "}}";
	}
	telemetry << "]";

	telemetry << ",\"candidates\":[";
	for (size_t i = 0; i < s.candidates.size(); ++i) {
		if (i > 0)
			telemetry << ",";
		telemetry << "{\"ang_deg\":" << s.candidates[i].angle_deg
				  << ",\"dist_mm\":" << s.candidates[i].distance_mm
				  << ",\"score\":" << s.candidates[i].score
				  << ",\"terms\":{\"distance\":" << s.candidates[i].score
				  << "}}";
	}
	telemetry << "]";

	if (status_.valid) {
		telemetry << ",\"esp_status\":{\"auto_active\":"
				  << (unsigned)status_.auto_active
				  << ",\"faults\":" << status_.faults
				  << ",\"speed_mm_s\":" << status_.speed_mm_s
				  << ",\"steer_cdeg\":" << status_.steer_cdeg
				  << ",\"age_ms\":" << status_.age_ms << "}";
	} else {
		telemetry << ",\"esp_status\":null";
	}

	telemetry << "}";
	MC_LOGI("telemetry", telemetry.str());
}

void TelemetryEmitter::emitOverrideEvent_(const TelemetrySample &s) {
	if (s.override_kind == last_override_)
		return;
	std::ostringstream event;
	event << "{\"t_us\":" << s.ts_us << ",\"type\":\"OVERRIDE\""
		  << ",\"run_id\":\"" << s.run_id << "\",\"tick\":" << s.tick
		  << ",\"scan_id\":" << s.scan_id << ",\"kind\":\"" << s.override_kind
		  << "\",\"detail\":\"" << s.override_detail
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
	if (s.override_detail != "NONE")
		l2 << " (" << s.override_detail << ")";

	std::ostringstream l3;
	l3 << "path_obst=" << s.path_obst_mm
	   << "mm  min_handle=" << s.min_handle_angle_deg << "deg@"
	   << s.min_handle_dist_mm << "mm";
	if (s.front_dist_mm)
		l3 << "  front=" << *s.front_dist_mm << "mm";
	if (s.side_dist_mm)
		l3 << "  side=" << *s.side_dist_mm << "mm";

	std::ostringstream l4;
	l4 << std::fixed << std::setprecision(2);
	l4 << "cmd: v=" << s.base_speed << "->" << s.limited_speed
	   << "  steer=" << s.raw_steer_deg << "->" << s.steer_deg << "deg"
	   << "  curve=" << s.curve_ratio << "  sf=" << s.speed_factor;

	std::ostringstream l5;
	l5 << "mode=" << s.mode << " map=" << s.map_state;
	if (s.scan_age_ms)
		l5 << " scan_age=" << *s.scan_age_ms << "ms";
	if (s.best_delta_deg)
		l5 << " d_best=" << *s.best_delta_deg << "deg";
	l5 << " tick=" << s.tick;
	if (s.run_id.size() >= 6)
		l5 << " run=" << s.run_id.substr(s.run_id.size() - 6);

	std::ostringstream l6;
	l6 << "latency p="
	   << (s.planner_latency_ms ? std::to_string(*s.planner_latency_ms) : "NA")
	   << "ms c="
	   << (s.control_latency_ms ? std::to_string(*s.control_latency_ms) : "NA")
	   << "ms ttl=" << (s.ttl_ms ? std::to_string(*s.ttl_ms) : "NA");
	if (status_.valid) {
		l6 << " auto=" << (unsigned)status_.auto_active << " faults=0x"
		   << std::hex << status_.faults << std::dec
		   << " age=" << status_.age_ms << "ms";
	}

	std::ostringstream l7;
	if (metrics_.valid) {
		l7 << "metrics: temp=" << (metrics_.cpu_temp_cdeg / 100.0f)
		   << "C cpu=" << (metrics_.cpu_usage_permille / 10.0f)
		   << "% mem=" << (metrics_.mem_used_kb / 1024.0f) << "/"
		   << (metrics_.mem_total_kb / 1024.0f) << "MB";
	} else {
		l7 << "metrics: NA";
	}

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
				  << line(l5.str()) << "\n"
				  << line(l6.str()) << "\n"
				  << line(l7.str()) << "\n"
				  << bot;
		std::cout << "\x1b[9A";
		ui_initialized_ = true;
	} else {
		std::cout << "\x1b[9A";
		std::cout << "\x1b[2K\r" << top << "\n";
		std::cout << "\x1b[2K\r" << line(l1.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l2.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l3.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l4.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l5.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l6.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l7.str()) << "\n";
		std::cout << "\x1b[2K\r" << bot;
		std::cout << "\x1b[9A";
	}
	std::cout << std::flush;
}
