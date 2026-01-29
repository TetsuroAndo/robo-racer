#include <mc/core/Log.hpp>
#include <mc/core/Time.hpp>
#include <mc/ipc/UdsSeqPacket.hpp>
#include <mc/proto/Proto.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

struct MetricsSample {
	uint16_t cpu_temp_cdeg = 0;
	uint16_t cpu_usage_permille = 0;
	uint32_t mem_used_kb = 0;
	uint32_t mem_total_kb = 0;
};

bool read_cpu_temp_cdeg(uint16_t &out) {
	std::ifstream ifs("/sys/class/thermal/thermal_zone0/temp");
	if (!ifs.is_open())
		return false;
	int temp_mdeg = 0;
	ifs >> temp_mdeg;
	if (!ifs.good())
		return false;
	out = (uint16_t)(temp_mdeg / 10);
	return true;
}

bool read_mem_kb(uint32_t &used_kb, uint32_t &total_kb) {
	std::ifstream ifs("/proc/meminfo");
	if (!ifs.is_open())
		return false;
	std::string key;
	uint64_t value = 0;
	std::string unit;
	uint64_t mem_total = 0;
	uint64_t mem_avail = 0;
	while (ifs >> key >> value >> unit) {
		if (key == "MemTotal:")
			mem_total = value;
		else if (key == "MemAvailable:")
			mem_avail = value;
	}
	if (mem_total == 0)
		return false;
	total_kb = (uint32_t)mem_total;
	used_kb = (uint32_t)(mem_total - mem_avail);
	return true;
}

struct CpuTimes {
	uint64_t idle = 0;
	uint64_t total = 0;
};

bool read_cpu_times(CpuTimes &out) {
	std::ifstream ifs("/proc/stat");
	if (!ifs.is_open())
		return false;
	std::string cpu;
	uint64_t user = 0, nice = 0, system = 0, idle = 0, iowait = 0;
	uint64_t irq = 0, softirq = 0, steal = 0;
	ifs >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >>
		steal;
	if (!ifs.good())
		return false;
	const uint64_t idle_all = idle + iowait;
	const uint64_t non_idle = user + nice + system + irq + softirq + steal;
	out.idle = idle_all;
	out.total = idle_all + non_idle;
	return true;
}

uint16_t compute_cpu_permille(const CpuTimes &prev, const CpuTimes &cur) {
	const uint64_t totald = cur.total - prev.total;
	const uint64_t idled = cur.idle - prev.idle;
	if (totald == 0)
		return 0;
	const uint64_t used = totald - idled;
	uint64_t permille = (used * 1000u) / totald;
	if (permille > 1000)
		permille = 1000;
	return (uint16_t)permille;
}

bool send_metrics(mc::ipc::UdsClient &ipc, const MetricsSample &m,
				  uint32_t ts_ms) {
	mc::proto::MetricsPayload payload{};
	payload.ts_ms = ts_ms;
	payload.cpu_temp_cdeg = m.cpu_temp_cdeg;
	payload.cpu_usage_permille = m.cpu_usage_permille;
	payload.mem_used_kb = m.mem_used_kb;
	payload.mem_total_kb = m.mem_total_kb;

	uint8_t enc[mc::proto::MAX_FRAME_ENCODED];
	size_t enc_len = 0;
	if (!mc::proto::PacketWriter::build(
			enc, sizeof(enc), enc_len, mc::proto::Type::IPC_METRICS, 0, 0,
			reinterpret_cast< const uint8_t * >(&payload), sizeof(payload))) {
		return false;
	}
	return ::send(ipc.fd(), enc, enc_len, 0) == (ssize_t)enc_len;
}

} // namespace

int main(int argc, char **argv) {
	uint32_t interval_ms = 1000;
	std::string log_path;
	std::string sock_path;

	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--interval-ms" && i + 1 < argc) {
			interval_ms = (uint32_t)std::atoi(argv[++i]);
		} else if (a == "--log" && i + 1 < argc) {
			log_path = argv[++i];
		} else if (a == "--sock" && i + 1 < argc) {
			sock_path = argv[++i];
		}
	}
	if (interval_ms == 0)
		interval_ms = 1000;

	auto &logger = mc::core::Logger::instance();
	if (!log_path.empty()) {
		logger.addSink(std::make_shared< mc::core::FileSink >(log_path));
	}

	mc::ipc::UdsClient ipc;
	bool has_ipc = false;
	if (!sock_path.empty()) {
		has_ipc = ipc.connect(sock_path);
	}

	CpuTimes prev{};
	(void)read_cpu_times(prev);
	while (true) {
		MetricsSample m{};
		read_cpu_temp_cdeg(m.cpu_temp_cdeg);
		read_mem_kb(m.mem_used_kb, m.mem_total_kb);
		CpuTimes cur{};
		if (read_cpu_times(cur)) {
			m.cpu_usage_permille = compute_cpu_permille(prev, cur);
			prev = cur;
		}

		logger.log(mc::core::LogLevel::Info, "metrics",
				   "temp_cdeg=" + std::to_string(m.cpu_temp_cdeg) +
					   " cpu_permille=" + std::to_string(m.cpu_usage_permille) +
					   " mem_kb=" + std::to_string(m.mem_used_kb) + "/" +
					   std::to_string(m.mem_total_kb));

		if (has_ipc) {
			const uint32_t ts_ms = mc::core::Time::ms();
			(void)send_metrics(ipc, m, ts_ms);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
	}

	logger.shutdown();
	return 0;
}
