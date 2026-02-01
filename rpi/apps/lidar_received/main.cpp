#include "config/Config.h"
#include "lidar_recieiver.h"

#include <mc/core/Log.hpp>

#include <cerrno>
#include <memory>
#include <string>
#include <sys/stat.h>

static sl::ILidarDriver *g_lidar = 0;
static sl::IChannel *g_ch = 0;

static int g_shm_fd = -1;
static ShmLidarScanData *g_shm = 0;
static sem_t *g_sem = SEM_FAILED;
static bool g_shm_owner = false;
static bool g_sem_owner = false;

static const char *SHM_NAME = "/lidar_scan";
static const char *SEM_NAME = "/lidar_scan_sem";

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	(void)sig;
	g_stop = 1;
}

static void cleanup_lidar_partial() {
	// Driver cleanup
	if (g_lidar) {
		g_lidar->stop();
		g_lidar->disconnect();
		delete g_lidar;
		g_lidar = 0;
	}
	// Channel cleanup
	if (g_ch) {
		delete g_ch;
		g_ch = 0;
	}
}

bool start_lidar() {
	const char *dev = cfg::DEFAULT_LIDAR_DEVICE;
	int baud = cfg::DEFAULT_LIDAR_BAUD;

	auto channelRes = sl::createSerialPortChannel(std::string(dev), baud);
	if (!channelRes)
		return (MC_LOGE("lidar_received",
						"createSerialPortChannel failed dev=" +
							std::string(dev) + " baud=" + std::to_string(baud)),
				cleanup_lidar_partial(), false);
	g_ch = channelRes.value;

	auto lidarRes = sl::createLidarDriver();
	if (!lidarRes)
		return (MC_LOGE("lidar_received", "createLidarDriver failed"),
				cleanup_lidar_partial(), false);
	g_lidar = lidarRes.value;

	if (SL_IS_FAIL(g_lidar->connect(g_ch)))
		return (MC_LOGE("lidar_received", "lidar connect failed"),
				cleanup_lidar_partial(), false);

	sl_lidar_response_device_info_t info;
	if (SL_IS_FAIL(g_lidar->getDeviceInfo(info)))
		return (MC_LOGE("lidar_received", "getDeviceInfo failed"),
				cleanup_lidar_partial(), false);

	sl_lidar_response_device_health_t health;
	if (SL_IS_FAIL(g_lidar->getHealth(health)))
		return (MC_LOGE("lidar_received", "getHealth failed"),
				cleanup_lidar_partial(), false);

	if (SL_IS_FAIL(g_lidar->setMotorSpeed()))
		return (MC_LOGE("lidar_received", "setMotorSpeed failed"),
				cleanup_lidar_partial(), false);

	sl::LidarScanMode scanMode;
	if (SL_IS_FAIL(g_lidar->startScan(false, true, 0, &scanMode)))
		return (MC_LOGE("lidar_received", "startScan failed"),
				cleanup_lidar_partial(), false);
	MC_LOGI("lidar_received", "lidar started dev=" + std::string(dev) +
								  " baud=" + std::to_string(baud));
	return (true);
}

void cleanup_sem() {
	// safe multiple calls
	if (g_sem != SEM_FAILED && g_sem != 0) {
		sem_close(g_sem);
		g_sem = SEM_FAILED;
	}

	if (g_shm && g_shm != MAP_FAILED) {
		munmap(g_shm, sizeof(ShmLidarScanData));
		g_shm = 0;
	}

	if (g_shm_fd >= 0) {
		close(g_shm_fd);
		g_shm_fd = -1;
	}

	if (g_sem_owner) {
		sem_unlink(SEM_NAME);
		g_sem_owner = false;
	}
	if (g_shm_owner) {
		shm_unlink(SHM_NAME);
		g_shm_owner = false;
	}
}

bool init_sem() {
	g_shm_fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0666);
	if (g_shm_fd < 0) {
		if (errno != EEXIST)
			return (MC_LOGE("lidar_received", "shm_open failed"), cleanup_sem(),
					false);
		g_shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
		if (g_shm_fd < 0)
			return (MC_LOGE("lidar_received", "shm_open existing failed"),
					cleanup_sem(), false);
		g_shm_owner = false;
	} else {
		g_shm_owner = true;
	}

	if (g_shm_owner) {
		if (ftruncate(g_shm_fd, (off_t)sizeof(ShmLidarScanData)) < 0)
			return (MC_LOGE("lidar_received", "ftruncate failed"),
					cleanup_sem(), false);
	}

	void *p = mmap(NULL, sizeof(ShmLidarScanData), PROT_READ | PROT_WRITE,
				   MAP_SHARED, g_shm_fd, 0);
	if (p == MAP_FAILED)
		return (MC_LOGE("lidar_received", "mmap failed"), cleanup_sem(), false);

	g_shm = (ShmLidarScanData *)p;

	g_sem = sem_open(SEM_NAME, O_CREAT | O_EXCL, 0666, 1);
	if (g_sem == SEM_FAILED) {
		if (errno != EEXIST)
			return (MC_LOGE("lidar_received", "sem_open failed"), cleanup_sem(),
					false);
		g_sem = sem_open(SEM_NAME, 0);
		if (g_sem == SEM_FAILED)
			return (MC_LOGE("lidar_received", "sem_open existing failed"),
					cleanup_sem(), false);
		g_sem_owner = false;
	} else {
		g_sem_owner = true;
	}

	if (g_shm_owner || g_sem_owner) {
		while (sem_wait(g_sem) == -1) {
			if (errno == EINTR) {
				if (g_stop)
					return (MC_LOGE("lidar_received",
									"sem_wait interrupted during init"),
							cleanup_sem(), false);
				continue;
			}
			return (MC_LOGE("lidar_received", "sem_wait failed"), cleanup_sem(),
					false);
		}
		g_shm->seq = LIDAR_NOUPDATED;
		for (int i = 0; i < 181; ++i)
			g_shm->distance_mm[i] = 0;
		sem_post(g_sem);
	}

	MC_LOGI("lidar_received", std::string("shared memory ready owner=") +
								  (g_shm_owner ? "1" : "0") +
								  " sem_owner=" + (g_sem_owner ? "1" : "0"));
	return (true);
}

static inline float normalize_deg_180(float deg) {
	while (deg >= 180.f)
		deg -= 360.f;
	while (deg < -180.f)
		deg += 360.f;
	return (deg);
}

static bool receiveScanData(LidarScanData *scan_data) {
	sl_lidar_response_measurement_node_hq_t nodes[cfg::LIDAR_NODE_MAX];
	size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);

	sl_result ans = g_lidar->grabScanDataHq(nodes, nodeCount);
	if (!SL_IS_OK(ans) || nodeCount == 0)
		return (false);

	// 上記の取得だけではソートされていないため以下の関数でソートする
	ans = g_lidar->ascendScanData(nodes, nodeCount);
	if (!SL_IS_OK(ans))
		return (false);

	for (int i = -90; i <= 90; ++i)
		scan_data->setDistance(i, 0);

	// 同じビンに複数値の候補がある場合は距離が近いほうを採用する
	for (size_t i = 0; i < nodeCount; i++) {
		if (nodes[i].dist_mm_q2 == 0)
			continue;

		float angle = (float)nodes[i].angle_z_q14 * 90.f / 16384.f;
		angle = normalize_deg_180(angle);
		int32_t angle_i = static_cast< int32_t >(std::roundf(angle));
		if (angle_i < -90 || angle_i > 90)
			continue;

		int32_t dist = (int32_t)(nodes[i].dist_mm_q2 >> 2);
		if (dist <= 0)
			continue;

		int32_t old_data = scan_data->getDistance(angle_i);
		if (old_data > dist || old_data <= 0)
			scan_data->setDistance(angle_i, dist);
	}
	return (true);
}

void write_to_mem(const LidarScanData &scan_data) {
	while (sem_wait(g_sem) == -1) {
		if (errno == EINTR)
			continue;
		MC_LOGE("lidar_received", "sem_wait failed in write_to_mem");
		return;
	}
	for (int i = -90; i <= 90; ++i)
		g_shm->distance_mm[i + 90] = scan_data.getDistance(i);
	if (g_shm->seq == UINT32_MAX)
		g_shm->seq = 1;
	else
		g_shm->seq++;
	sem_post(g_sem);
}

static void ensure_dir_for(const std::string &path) {
	if (path.empty())
		return;
	const size_t pos = path.find_last_of('/');
	if (pos == std::string::npos || pos == 0)
		return;
	const std::string dir = path.substr(0, pos);
	const int rc = mkdir(dir.c_str(), 0755);
	if (rc == 0 || errno == EEXIST)
		return;
	MC_LOGW("lidar_received", "mkdir failed for log dir: " + dir);
}

int main(int argc, char **argv) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	std::string log_path = lidar_received_cfg::DEFAULT_LOG;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--log" && i + 1 < argc) {
			log_path = argv[++i];
		}
	}

	auto &logger = mc::core::Logger::instance();
	if (!log_path.empty()) {
		ensure_dir_for(log_path);
		logger.addSink(std::make_shared< mc::core::FileSink >(log_path));
	}

	MC_LOGI("lidar_received", "starting");

	if (!start_lidar() || !init_sem()) {
		logger.shutdown();
		return (1);
	}

	LidarScanData scan_data;
	while (!g_stop) {
		if (receiveScanData(&scan_data))
			write_to_mem(scan_data);
		else
			usleep(1000);
	}

	// 終了処理
	MC_LOGI("lidar_received", "shutdown");
	cleanup_lidar_partial();
	// 共有メモリおよびセマフォのクリーンアップは cleanup_sem() に集約
	cleanup_sem();
	logger.shutdown();
	return (0);
}
