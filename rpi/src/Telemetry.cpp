#include "Telemetry.h"

#include "config/Config.h"
#include "mc/core/Log.hpp"
#include "mc/core/Time.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {

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

std::string trimVisible(const std::string &s, size_t max_vis) {
	size_t vis = 0;
	std::string out;
	out.reserve(s.size());
	for (size_t i = 0; i < s.size(); ++i) {
		if (s[i] == '\x1b' && i + 1 < s.size() && s[i + 1] == '[') {
			// copy ANSI sequence without counting toward visible length
			out.push_back(s[i++]);
			out.push_back(s[i++]);
			while (i < s.size()) {
				out.push_back(s[i]);
				if (s[i] == 'm')
					break;
				++i;
			}
			continue;
		}
		if (vis >= max_vis)
			break;
		out.push_back(s[i]);
		++vis;
	}
	return out;
}

std::string fitVisible(const std::string &s, size_t width) {
	std::string out = trimVisible(s, width);
	const size_t vis = visibleLen(out);
	if (vis < width)
		out += std::string(width - vis, ' ');
	return out;
}

int glyphPriority(char c) {
	if (c == '*' || c == 'B' || c == 'S')
		return 3;
	if (c == '!')
		return 2;
	if (c == ' ')
		return 0;
	return 1;
}

int intensityRank(char c) {
	const char *levels = " .:-=+*#@!";
	for (int i = 0; levels[i] != '\0'; ++i) {
		if (levels[i] == c)
			return i;
	}
	return 0;
}

std::string colorWrap(const std::string &s, Severity sev) {
	switch (sev) {
	case Severity::Safe:
		return "\x1b[32m" + s + "\x1b[0m";
	case Severity::Warn:
		return "\x1b[33m" + s + "\x1b[0m";
	case Severity::Crit:
		return "\x1b[31m" + s + "\x1b[0m";
	}
	return s;
}

std::string bar(float ratio, size_t width) {
	if (ratio < 0.0f)
		ratio = 0.0f;
	if (ratio > 1.0f)
		ratio = 1.0f;
	const size_t fill = (size_t)std::round(ratio * (float)width);
	return "[" + std::string(fill, '#') + std::string(width - fill, '-') + "]";
}

std::string sparkline(const std::vector< float > &vals, float min_v,
					  float max_v, size_t width) {
	static const char *k = " .:-=+*#@";
	const size_t levels = 8;
	std::string out;
	out.reserve(width);
	if (vals.empty()) {
		out.assign(width, ' ');
		return out;
	}
	const float denom = (max_v - min_v);
	const size_t start = (vals.size() > width) ? (vals.size() - width) : 0;
	const size_t pad = (vals.size() < width) ? (width - vals.size()) : 0;
	out.assign(pad, ' ');
	for (size_t i = start; i < vals.size(); ++i) {
		float v = vals[i];
		float n = (denom <= 0.0f) ? 0.5f : (v - min_v) / denom;
		if (n < 0.0f)
			n = 0.0f;
		if (n > 1.0f)
			n = 1.0f;
		const size_t idx = (size_t)std::round(n * (float)levels);
		out.push_back(k[idx]);
	}
	return out;
}

} // namespace

TelemetryEmitter::TelemetryEmitter() {
	running_.store(true);
	metrics_thread_ = std::thread(&TelemetryEmitter::metricsLoop_, this);
}

TelemetryEmitter::~TelemetryEmitter() {
	running_.store(false);
	if (metrics_thread_.joinable())
		metrics_thread_.join();
	shutdownUi();
}

void TelemetryEmitter::setRateHz(double hz) {
	if (hz <= 0.0) {
		telemetry_interval_us_ = 0;
		return;
	}
	telemetry_interval_us_ = (uint64_t)(1000000.0 / hz);
}

void TelemetryEmitter::setMetricsLogPath(std::string path) {
	std::lock_guard< std::mutex > lk(metrics_mtx_);
	metrics_log_path_ = std::move(path);
}

void TelemetryEmitter::emit(const TelemetrySample &s) {
	const uint64_t now_us = s.ts_us;
	emitOverrideEvent_(s);

	if (telemetry_interval_us_ == 0 ||
		now_us - last_telemetry_us_ >= telemetry_interval_us_) {
		emitJson_(s);
		last_telemetry_us_ = now_us;
	}
	if (ui_enabled_ && (telemetry_interval_us_ == 0 ||
						now_us - last_ui_us_ >= telemetry_interval_us_)) {
		emitUi_(s);
		last_ui_us_ = now_us;
	}
}

void TelemetryEmitter::emitNoLidar(uint64_t ts_us, const std::string &run_id,
								   uint64_t tick, uint64_t scan_id) {
	std::ostringstream json;
	json << "{\"t_us\":" << ts_us << ",\"type\":\"NO_LIDAR\""
		 << ",\"run_id\":\"" << run_id << "\",\"tick\":" << tick
		 << ",\"scan_id\":" << scan_id
		 << ",\"mode\":\"UNKNOWN\",\"cmd\":{\"v\":0,\"steer_deg\":0}}";
	MC_LOGW("event", json.str());
	pushEvent_("NO_LIDAR");
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
	std::string path;
	{
		std::lock_guard< std::mutex > lk(metrics_mtx_);
		path = metrics_log_path_;
	}
	if (path.empty())
		return;
	const uint64_t now_us = mc::core::Time::us();
	if (now_us - metrics_last_read_us_ < 1000 * 1000)
		return;
	metrics_last_read_us_ = now_us;

	std::ifstream ifs(path, std::ios::binary);
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

	{
		std::lock_guard< std::mutex > lk(metrics_mtx_);
		metrics_.valid = true;
		metrics_.cpu_temp_cdeg = (uint16_t)temp;
		metrics_.cpu_usage_permille = (uint16_t)cpu;
		metrics_.mem_used_kb = (uint32_t)mem_used;
		metrics_.mem_total_kb = (uint32_t)mem_total;
	}
}

void TelemetryEmitter::metricsLoop_() {
	using namespace std::chrono_literals;
	while (running_.load()) {
		refreshMetrics_();
		std::this_thread::sleep_for(1s);
	}
}

void TelemetryEmitter::pushEvent_(std::string e) {
	if (e.empty())
		return;
	events_.push_back(std::move(e));
	const size_t max_events = 5;
	if (events_.size() > max_events)
		events_.erase(events_.begin(),
					  events_.begin() + (events_.size() - max_events));
}

void TelemetryEmitter::emitJson_(const TelemetrySample &s) {
	std::ostringstream telemetry;
	telemetry << std::fixed << std::setprecision(2);
	telemetry << "{\"t_us\":" << s.ts_us << ",\"type\":\"telemetry\""
			  << ",\"run_id\":\"" << s.run_id << "\",\"tick\":" << s.tick
			  << ",\"scan_id\":" << s.scan_id << ",\"mode\":\"" << s.mode
			  << "\",\"map_state\":\"" << s.map_state << "\""
			  << ",\"best\":{\"ang_deg\":" << s.best_angle_deg
			  << ",\"dist_mm\":" << s.best_dist_mm
			  << ",\"score\":" << s.best_score << ",\"delta_deg\":"
			  << (s.best_delta_deg ? std::to_string(*s.best_delta_deg) : "null")
			  << ",\"terms\":{\"distance\":" << s.best_score;
	if (level_ == TelemetryLevel::Full) {
		telemetry
			<< ",\"obstacle\":"
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
			<< (s.score_map_conf ? std::to_string(*s.score_map_conf) : "null");
	}
	telemetry << "}}"
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
			  << ",\"override\":{\"kind\":\"" << s.override_kind
			  << "\",\"detail\":\"" << s.override_detail << "\"}"
			  << ",\"scan_age_ms\":"
			  << (s.scan_age_ms ? std::to_string(*s.scan_age_ms) : "null")
			  << ",\"latency_ms\":{\"planner\":"
			  << (s.planner_latency_ms ? std::to_string(*s.planner_latency_ms)
									   : "null")
			  << ",\"control\":"
			  << (s.control_latency_ms ? std::to_string(*s.control_latency_ms)
									   : "null")
			  << "}"
			  << ",\"ttl_ms\":"
			  << (s.ttl_ms ? std::to_string(*s.ttl_ms) : "null")
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

	if (level_ == TelemetryLevel::Full && s.include_candidates) {
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
	}

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
	pushEvent_("OVERRIDE " + s.override_kind);
	last_override_ = s.override_kind;
}

void TelemetryEmitter::emitUi_(const TelemetrySample &s) {
	auto push_hist = [](std::vector< float > &h, float v) {
		h.push_back(v);
		if (h.size() > cfg::TELEMETRY_SPARK_LEN)
			h.erase(h.begin());
	};

	MetricsCache metrics;
	{
		std::lock_guard< std::mutex > lk(metrics_mtx_);
		metrics = metrics_;
	}

	int path_mm = s.path_obst_mm;
	if (path_mm < 0 || path_mm >= (std::numeric_limits< int >::max() / 2)) {
		path_mm = cfg::TELEMETRY_DIST_BAR_MAX_MM;
	}
	if (path_mm > cfg::TELEMETRY_DIST_BAR_MAX_MM)
		path_mm = cfg::TELEMETRY_DIST_BAR_MAX_MM;

	push_hist(spark_best_, s.best_angle_deg);
	push_hist(spark_speed_, (float)s.limited_speed);
	push_hist(spark_dist_, (float)path_mm);

	float best_std = 0.0f;
	if (spark_best_.size() >= 2) {
		float sum = 0.0f;
		for (float v : spark_best_)
			sum += v;
		const float mean = sum / (float)spark_best_.size();
		float var = 0.0f;
		for (float v : spark_best_) {
			const float d = v - mean;
			var += d * d;
		}
		var /= (float)spark_best_.size();
		best_std = std::sqrt(var);
	}

	float jerk = 0.0f;
	if (s.best_delta_deg) {
		if (has_last_delta_) {
			jerk = std::fabs(*s.best_delta_deg - last_delta_);
		}
		last_delta_ = *s.best_delta_deg;
		has_last_delta_ = true;
	}

	Severity cpu_sev = Severity::Safe;
	Severity temp_sev = Severity::Safe;
	if (metrics.valid) {
		if (metrics.cpu_usage_permille >= cfg::TELEMETRY_CPU_CRIT_PERMILLE)
			cpu_sev = Severity::Crit;
		else if (metrics.cpu_usage_permille >= cfg::TELEMETRY_CPU_WARN_PERMILLE)
			cpu_sev = Severity::Warn;
		if (metrics.cpu_temp_cdeg >= cfg::TELEMETRY_TEMP_CRIT_CDEG)
			temp_sev = Severity::Crit;
		else if (metrics.cpu_temp_cdeg >= cfg::TELEMETRY_TEMP_WARN_CDEG)
			temp_sev = Severity::Warn;
	}

	Severity dist_sev = Severity::Safe;
	if (s.path_obst_mm <= cfg::PROCESS_MIN_DIST_STOP_MM) {
		dist_sev = Severity::Crit;
	} else if (s.path_obst_mm < cfg::PROCESS_MIN_DIST_SAFE_MM) {
		dist_sev = Severity::Warn;
	}

	Severity map_sev = Severity::Safe;
	if (s.map_state == "INVALID" || s.map_state == "ERROR")
		map_sev = Severity::Crit;
	else if (s.map_state == "UNKNOWN" || s.map_state == "PARTIAL" ||
			 s.map_state == "BUILDING")
		map_sev = Severity::Warn;
	Severity loc_sev = Severity::Warn;
	if (s.scan_age_ms) {
		if (*s.scan_age_ms >= cfg::TELEMETRY_SCAN_AGE_CRIT_MS)
			loc_sev = Severity::Crit;
		else if (*s.scan_age_ms >= cfg::TELEMETRY_SCAN_AGE_WARN_MS)
			loc_sev = Severity::Warn;
		else
			loc_sev = Severity::Safe;
	}

	float sens_ratio = -1.0f;
	Severity sens_sev = Severity::Warn;
	if (s.lidar_points && s.lidar_expected && *s.lidar_expected > 0) {
		sens_ratio = (float)*s.lidar_points / (float)*s.lidar_expected;
		if (sens_ratio < 0.5f)
			sens_sev = Severity::Crit;
		else if (sens_ratio < 0.8f)
			sens_sev = Severity::Warn;
		else
			sens_sev = Severity::Safe;
	}

	Severity ctrl_sev = Severity::Safe;
	bool ttl_expired = false;
	if (status_.valid && s.ttl_ms) {
		const float ttl = (float)*s.ttl_ms;
		if ((float)status_.age_ms >= ttl * cfg::TELEMETRY_TTL_CRIT_FACTOR) {
			ctrl_sev = Severity::Crit;
			ttl_expired = true;
		} else if ((float)status_.age_ms >=
				   ttl * cfg::TELEMETRY_TTL_WARN_FACTOR) {
			ctrl_sev = Severity::Warn;
			ttl_expired = true;
		}
	}
	if (s.override_kind == "STOP")
		ctrl_sev = Severity::Crit;
	else if (s.override_kind != "NONE")
		ctrl_sev = std::max(ctrl_sev, Severity::Warn);
	if (status_.valid && status_.faults != 0)
		ctrl_sev = std::max(ctrl_sev, Severity::Warn);
	if (s.planner_latency_ms &&
		*s.planner_latency_ms >= cfg::TELEMETRY_LATENCY_CRIT_MS)
		ctrl_sev = std::max(ctrl_sev, Severity::Crit);
	else if (s.planner_latency_ms &&
			 *s.planner_latency_ms >= cfg::TELEMETRY_LATENCY_WARN_MS)
		ctrl_sev = std::max(ctrl_sev, Severity::Warn);

	std::ostringstream l0;
	l0 << "MAP " << colorWrap(s.map_state, map_sev) << " | MODE " << s.mode;
	if (s.scan_age_ms)
		l0 << " | scan_age " << *s.scan_age_ms << "ms";
	l0 << " | run "
	   << (s.run_id.size() >= 6 ? s.run_id.substr(s.run_id.size() - 6)
								: s.run_id)
	   << " | tick " << s.tick;

	std::ostringstream l1;
	l1 << "BADGES " << colorWrap("MAP", map_sev) << "/"
	   << colorWrap("LOC", loc_sev) << "/" << colorWrap("SENS", sens_sev) << "/"
	   << colorWrap("CTRL", ctrl_sev);
	l1 << " | d_best="
	   << (s.best_delta_deg ? std::to_string(*s.best_delta_deg) : "NA")
	   << "deg std=" << std::fixed << std::setprecision(1) << best_std
	   << " jerk=" << std::fixed << std::setprecision(1) << jerk;
	const double hz = (telemetry_interval_us_ > 0)
						  ? (1000000.0 / (double)telemetry_interval_us_)
						  : 0.0;
	l1 << " | lvl=" << (level_ == TelemetryLevel::Full ? "full" : "basic");
	if (hz > 0.0)
		l1 << " hz=" << std::fixed << std::setprecision(1) << hz;

	const size_t frame_width = 110;
	const size_t scale_w = TELEMETRY_COMPASS_BINS;
	const size_t compass_pad = 0;
	const std::string compass_left(compass_pad, ' ');

	auto posFromAngle = [&](float angle_deg) {
		float clamped = angle_deg;
		if (clamped < -90.0f)
			clamped = -90.0f;
		if (clamped > 90.0f)
			clamped = 90.0f;
		const float ratio = (clamped + 90.0f) / 180.0f;
		const int pos = (int)std::lround(ratio * (float)(scale_w - 1));
		return std::max(0, std::min((int)scale_w - 1, pos));
	};

	const int max_dist = cfg::TELEMETRY_DIST_BAR_MAX_MM;
	std::ostringstream comp_title;
	comp_title << "COMPASS Fan (max " << std::fixed << std::setprecision(1)
			   << (max_dist / 1000.0f)
			   << "m, near<=" << cfg::TELEMETRY_NEAR_EMPH_MM
			   << "mm) B=best T=target A=applied";
	const std::string top_compass =
		compass_left + "+" + fitVisible(comp_title.str(), scale_w) + "+";

	std::vector< std::string > fan_rows;
	fan_rows.reserve(cfg::TELEMETRY_COMPASS_ROWS);
	const size_t rows = cfg::TELEMETRY_COMPASS_ROWS;
	const size_t min_width =
		std::min(cfg::TELEMETRY_COMPASS_MIN_WIDTH, TELEMETRY_COMPASS_BINS);
	std::vector< std::string > grid(rows,
									std::string(TELEMETRY_COMPASS_BINS, ' '));

	if (s.lidar_dist_valid) {
		static const char *k = " .:-=+*#@";
		const size_t kmax = 8;
		for (size_t c = 0; c < TELEMETRY_COMPASS_BINS; ++c) {
			const int raw_dist = s.lidar_dist_bins[c];
			const bool has_wall = (raw_dist >= 0 && raw_dist < max_dist);
			int dist = raw_dist;
			if (dist < 0)
				dist = max_dist;
			if (dist > max_dist)
				dist = max_dist;
			const float ratio = (max_dist > 0) ? (float)dist / max_dist : 0.0f;
			size_t fill_rows = (size_t)std::ceil(ratio * (float)rows);
			if (fill_rows > rows)
				fill_rows = rows;
			if (fill_rows == 0)
				continue;
			const size_t start_row = rows - fill_rows;
			if (has_wall) {
				for (size_t r = 0; r < rows; ++r) {
					grid[r][c] = '.';
				}
			}
			for (size_t r = start_row; r < rows; ++r) {
				const float t = (rows > 1)
									? (float)(rows - 1 - r) / (float)(rows - 1)
									: 0.0f;
				const size_t idx = (size_t)std::lround(t * (float)kmax);
				grid[r][c] = k[idx];
			}
			if (dist <= cfg::TELEMETRY_NEAR_EMPH_MM) {
				grid[start_row][c] = '!';
			}
		}
	}

	for (size_t r = 0; r < rows; ++r) {
		const float t =
			(rows > 1) ? (float)(rows - 1 - r) / (float)(rows - 1) : 0.0f;
		const size_t width = (size_t)std::lround(
			(float)min_width + t * (float)(TELEMETRY_COMPASS_BINS - min_width));
		const size_t left = (TELEMETRY_COMPASS_BINS - width) / 2;
		std::string row_line(TELEMETRY_COMPASS_BINS, ' ');
		for (size_t c = 0; c < TELEMETRY_COMPASS_BINS; ++c) {
			const size_t mapped =
				(width > 1)
					? (size_t)std::lround((float)c * (float)(width - 1) /
										  (float)(TELEMETRY_COMPASS_BINS - 1))
					: 0;
			const size_t col = left + mapped;
			if (col >= TELEMETRY_COMPASS_BINS)
				continue;
			const char in = grid[r][c];
			if (intensityRank(in) >= intensityRank(row_line[col])) {
				row_line[col] = in;
			}
		}
		fan_rows.push_back(row_line);
	}

	std::string labels(scale_w, ' ');
	auto place_label = [&](const std::string &txt, int pos) {
		int start = pos - (int)txt.size() / 2;
		if (start < 0)
			start = 0;
		if (start + (int)txt.size() > (int)scale_w)
			start = (int)scale_w - (int)txt.size();
		for (size_t i = 0; i < txt.size(); ++i) {
			if (start + (int)i >= 0 && start + (int)i < (int)scale_w) {
				labels[(size_t)(start + (int)i)] = txt[i];
			}
		}
	};
	const int ticks[] = {0, 10, 20, 30, 40, 50, 60};
	const char *tick_labels[] = {"-90", "-60", "-30", "0", "+30", "+60", "+90"};
	for (size_t i = 0; i < 7; ++i) {
		place_label(tick_labels[i], ticks[i]);
	}
	const std::string line_labels =
		compass_left + "|" + fitVisible(labels, scale_w) + "|";

	std::string marker(scale_w, '-');
	for (int pos : ticks) {
		if (pos >= 0 && pos < (int)scale_w)
			marker[(size_t)pos] = '|';
	}
	auto marker_prio = [](char c) {
		switch (c) {
		case 'A':
			return 3;
		case 'T':
			return 2;
		case 'B':
			return 1;
		default:
			return 0;
		}
	};
	auto place_marker = [&](float angle_deg, char c) {
		const int pos = posFromAngle(angle_deg);
		char &slot = marker[(size_t)pos];
		if (marker_prio(c) >= marker_prio(slot))
			slot = c;
	};
	place_marker(s.best_angle_deg, 'B');
	place_marker(s.raw_steer_deg, 'T');
	place_marker((float)s.steer_deg, 'A');
	std::string marker_colored;
	marker_colored.reserve(marker.size() * 4);
	for (char c : marker) {
		switch (c) {
		case 'B':
			marker_colored += "\x1b[32mB\x1b[0m";
			break;
		case 'T':
			marker_colored += "\x1b[33mT\x1b[0m";
			break;
		case 'A':
			marker_colored += "\x1b[36mA\x1b[0m";
			break;
		case '*':
			marker_colored += "\x1b[31m*\x1b[0m";
			break;
		case '|':
			marker_colored += "\x1b[90m|\x1b[0m";
			break;
		default:
			marker_colored.push_back(c);
			break;
		}
	}
	const std::string line_markers =
		compass_left + "|" + fitVisible(marker_colored, scale_w) + "|";

	const std::string bot_compass =
		compass_left + "+" + std::string(scale_w, '-') + "+";

	std::ostringstream l2;
	l2 << std::fixed << std::setprecision(1)
	   << "angles \x1b[32mB\x1b[0m=" << std::showpos << s.best_angle_deg
	   << "deg  \x1b[33mT\x1b[0m=" << s.raw_steer_deg
	   << "deg  \x1b[36mA\x1b[0m=" << (float)s.steer_deg << "deg";

	std::ostringstream l3;
	l3 << "top:";
	for (size_t i = 0; i < s.top_count; ++i) {
		l3 << " " << std::showpos << s.top[i].angle_deg << "(" << s.top[i].score
		   << ")";
	}
	l3 << "  override: " << colorOverride(s.override_kind);
	if (s.override_detail != "NONE")
		l3 << " (" << s.override_detail << ")";

	std::ostringstream l4;
	l4 << std::fixed << std::setprecision(2);
	l4 << "cmd: v=" << s.base_speed << "->" << s.limited_speed
	   << "  steer=" << s.raw_steer_deg << "->" << s.steer_deg << "deg"
	   << "  curve=" << s.curve_ratio << "  sf=" << s.speed_factor;

	std::ostringstream l5;
	l5 << "latency p="
	   << (s.planner_latency_ms ? std::to_string(*s.planner_latency_ms) : "NA")
	   << "ms c="
	   << (s.control_latency_ms ? std::to_string(*s.control_latency_ms) : "NA")
	   << "ms ttl=" << (s.ttl_ms ? std::to_string(*s.ttl_ms) : "NA");
	if (status_.valid) {
		l5 << " auto=" << (unsigned)status_.auto_active << " faults=0x"
		   << std::hex << status_.faults << std::dec
		   << " age=" << status_.age_ms << "ms";
	}

	std::ostringstream l6;
	l6 << "path_obst=" << s.path_obst_mm
	   << "mm  min_handle=" << s.min_handle_angle_deg << "deg@"
	   << s.min_handle_dist_mm << "mm";
	if (s.front_dist_mm)
		l6 << "  front=" << *s.front_dist_mm << "mm";
	if (s.side_dist_mm)
		l6 << "  side=" << *s.side_dist_mm << "mm";

	const std::string sp_angle =
		sparkline(spark_best_, -90.0f, 90.0f, cfg::TELEMETRY_SPARK_LEN);
	const std::string sp_speed =
		sparkline(spark_speed_, 0.0f, (float)cfg::PROCESS_MAX_SPEED,
				  cfg::TELEMETRY_SPARK_LEN);
	const std::string sp_dist =
		sparkline(spark_dist_, 0.0f, (float)cfg::TELEMETRY_DIST_BAR_MAX_MM,
				  cfg::TELEMETRY_SPARK_LEN);

	std::ostringstream l7;
	l7 << "spark ang:" << sp_angle << " spd:" << sp_speed
	   << " dst:" << colorWrap(sp_dist, dist_sev);

	std::ostringstream l8;
	l8 << std::fixed << std::setprecision(1);
	const size_t bar_w = cfg::TELEMETRY_BAR_WIDTH;
	if (metrics.valid) {
		const float cpu_ratio = (float)metrics.cpu_usage_permille / 1000.0f;
		const float temp_ratio =
			(float)metrics.cpu_temp_cdeg / (float)cfg::TELEMETRY_TEMP_CRIT_CDEG;
		l8 << "CPU " << colorWrap(bar(cpu_ratio, bar_w), cpu_sev) << " "
		   << (metrics.cpu_usage_permille / 10.0f) << "%  "
		   << "TEMP " << colorWrap(bar(temp_ratio, bar_w), temp_sev) << " "
		   << (metrics.cpu_temp_cdeg / 100.0f) << "C  ";
	} else {
		l8 << "CPU NA  TEMP NA  ";
	}
	const float dist_ratio =
		(float)path_mm / (float)cfg::TELEMETRY_DIST_BAR_MAX_MM;
	l8 << "DIST " << colorWrap(bar(dist_ratio, bar_w), dist_sev) << " "
	   << path_mm << "mm";
	if (metrics.valid) {
		l8 << "  MEM " << (metrics.mem_used_kb / 1024.0f) << "/"
		   << (metrics.mem_total_kb / 1024.0f) << "MB";
	}

	auto sev_name = [](Severity s) {
		switch (s) {
		case Severity::Safe:
			return "SAFE";
		case Severity::Warn:
			return "WARN";
		case Severity::Crit:
			return "CRIT";
		}
		return "UNK";
	};
	auto sev_rank = [](Severity s) {
		switch (s) {
		case Severity::Safe:
			return 0;
		case Severity::Warn:
			return 1;
		case Severity::Crit:
			return 2;
		}
		return 0;
	};

	const bool best_jump = s.best_delta_deg && std::fabs(*s.best_delta_deg) >=
												   cfg::TELEMETRY_BEST_JUMP_DEG;
	if (best_jump && !last_best_jump_) {
		pushEvent_("BEST_JUMP " + std::to_string(*s.best_delta_deg));
	}
	last_best_jump_ = best_jump;

	const bool slowdown = (s.speed_factor <= cfg::TELEMETRY_SLOWDOWN_SF);
	if (slowdown && !last_slowdown_) {
		pushEvent_("SLOWDOWN sf=" + std::to_string(s.speed_factor));
	}
	last_slowdown_ = slowdown;

	if (s.scan_age_ms && sev_rank(loc_sev) > sev_rank(last_loc_sev_)) {
		pushEvent_("SCAN_STALE " + std::to_string(*s.scan_age_ms) + "ms");
	}
	last_loc_sev_ = loc_sev;

	if (sev_rank(map_sev) > sev_rank(last_map_sev_)) {
		pushEvent_(std::string("MAP ") + sev_name(map_sev));
	}
	last_map_sev_ = map_sev;

	if (sev_rank(sens_sev) > sev_rank(last_sens_sev_)) {
		if (sens_ratio >= 0.0f) {
			const int miss_pct = (int)std::round((1.0f - sens_ratio) * 100.0f);
			pushEvent_("SENS_DROP miss=" + std::to_string(miss_pct) + "%");
		} else {
			pushEvent_("SENS_DROP");
		}
	}
	last_sens_sev_ = sens_sev;

	if (sev_rank(ctrl_sev) > sev_rank(last_ctrl_sev_)) {
		pushEvent_(std::string("CTRL ") + sev_name(ctrl_sev));
	}
	last_ctrl_sev_ = ctrl_sev;

	if (ttl_expired && !last_ttl_expired_) {
		pushEvent_("TTL_EXPIRED age=" + std::to_string(status_.age_ms) + "ms");
	}
	last_ttl_expired_ = ttl_expired;

	const bool faults_now = status_.valid && status_.faults != 0;
	if (faults_now && !last_faults_) {
		std::ostringstream f;
		f << "FAULTS 0x" << std::hex << status_.faults << std::dec;
		pushEvent_(f.str());
	}
	last_faults_ = faults_now;

	std::ostringstream l9;
	if (s.lidar_points && s.lidar_expected) {
		const float miss = 100.0f *
						   (float)(*s.lidar_expected - *s.lidar_points) /
						   (float)*s.lidar_expected;
		l9 << "lidar: " << *s.lidar_points << "/" << *s.lidar_expected
		   << " miss=" << std::fixed << std::setprecision(1) << miss << "%";
	} else {
		l9 << "lidar: NA";
	}
	l9 << " | heat:";
	std::string heat;
	heat.reserve(TELEMETRY_HEAT_BINS);
	for (size_t i = 0; i < s.heat_bins.size(); ++i) {
		float n = s.heat_bins[i];
		if (n < 0.0f)
			n = 0.0f;
		if (n > 1.0f)
			n = 1.0f;
		const size_t idx = (size_t)std::round(n * 8.0f);
		static const char *k = " .:-=+*#@";
		heat.push_back(k[idx]);
	}
	l9 << heat;

	std::ostringstream l10;
	if (!events_.empty()) {
		l10 << "events:";
		for (size_t i = 0; i < events_.size(); ++i) {
			if (i > 0)
				l10 << " | ";
			l10 << events_[i];
		}
	} else {
		l10 << "events: NA";
	}

	auto pad = [&](const std::string &sline) {
		const std::string trimmed = trimVisible(sline, frame_width - 2);
		const size_t vis = visibleLen(trimmed);
		if (vis >= frame_width - 2)
			return trimmed;
		return trimmed + std::string(frame_width - 2 - vis, ' ');
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

	const int frame_lines = 17 + (int)fan_rows.size();
	if (!ui_initialized_) {
		std::cout << "\x1b[?25l"; // hide cursor
		std::cout << top << "\n"
				  << line(l0.str()) << "\n"
				  << line(l1.str()) << "\n"
				  << line(top_compass) << "\n";
		for (const auto &row : fan_rows) {
			std::cout << line(compass_left + "|" + row + "|") << "\n";
		}
		std::cout << line(line_markers) << "\n"
				  << line(line_labels) << "\n"
				  << line(bot_compass) << "\n"
				  << line(l2.str()) << "\n"
				  << line(l3.str()) << "\n"
				  << line(l4.str()) << "\n"
				  << line(l5.str()) << "\n"
				  << line(l6.str()) << "\n"
				  << line(l7.str()) << "\n"
				  << line(l8.str()) << "\n"
				  << line(l9.str()) << "\n"
				  << line(l10.str()) << "\n"
				  << bot;
		std::cout << "\x1b[" << frame_lines << "A";
		ui_initialized_ = true;
	} else {
		std::cout << "\x1b[" << frame_lines << "A";
		std::cout << "\x1b[2K\r" << top << "\n";
		std::cout << "\x1b[2K\r" << line(l0.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l1.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(top_compass) << "\n";
		for (const auto &row : fan_rows) {
			std::cout << "\x1b[2K\r" << line(compass_left + "|" + row + "|")
					  << "\n";
		}
		std::cout << "\x1b[2K\r" << line(line_markers) << "\n";
		std::cout << "\x1b[2K\r" << line(line_labels) << "\n";
		std::cout << "\x1b[2K\r" << line(bot_compass) << "\n";
		std::cout << "\x1b[2K\r" << line(l2.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l3.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l4.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l5.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l6.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l7.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l8.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l9.str()) << "\n";
		std::cout << "\x1b[2K\r" << line(l10.str()) << "\n";
		std::cout << "\x1b[2K\r" << bot;
		std::cout << "\x1b[" << frame_lines << "A";
	}
	std::cout << std::flush;
}
