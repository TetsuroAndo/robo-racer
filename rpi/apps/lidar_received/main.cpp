#include "lidar_recieiver.h"

#include <string>

static sl::ILidarDriver *g_lidar = 0;
static sl::IChannel *g_ch = 0;

static int g_shm_fd = -1;
static ShmLidarScanData *g_shm = 0;
static sem_t *g_sem = SEM_FAILED;

static const char *SHM_NAME = "/lidar_scan";
static const char *SEM_NAME = "/lidar_scan_sem";

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	(void)sig;
#ifdef DEBUG
	std::cout << "\nSignal " << sig << " received, shutting down..." << std::endl;
#endif
	g_stop = 1;
}

static void cleanup_lidar_partial() {
	// Driver cleanup
	if (g_lidar) {
		delete g_lidar;
		g_lidar = 0;
	}
	// Channel cleanup
	if (g_ch) {
		delete g_ch;
		g_ch = 0;
	}
}

bool start_ridar() {
	const char *dev = cfg::DEFAULT_LIDAR_DEVICE;
	int baud = cfg::DEFAULT_LIDAR_BAUD;

	auto channelRes = sl::createSerialPortChannel(std::string(dev), baud);
	if (!channelRes)
		return (cleanup_lidar_partial(), false);
	g_ch = channelRes.value;

	auto lidarRes = sl::createLidarDriver();
	if (!lidarRes)
		return (cleanup_lidar_partial(), false);
	g_lidar = lidarRes.value;

	if (SL_IS_FAIL(g_lidar->connect(g_ch)))
		return (cleanup_lidar_partial(), false);

	sl_lidar_response_device_info_t info;
	if (SL_IS_FAIL(g_lidar->getDeviceInfo(info)))
		return (cleanup_lidar_partial(), false);

	sl_lidar_response_device_health_t health;
	if (SL_IS_FAIL(g_lidar->getHealth(health)))
		return (cleanup_lidar_partial(), false);

	if (SL_IS_FAIL(g_lidar->setMotorSpeed()))
		return (cleanup_lidar_partial(), false);

	sl::LidarScanMode scanMode;
	if (SL_IS_FAIL(g_lidar->startScan(false, true, 0, &scanMode)))
		return (cleanup_lidar_partial(), false);
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

	sem_unlink(SEM_NAME);
	shm_unlink(SHM_NAME);
}

bool init_sem() {
	g_shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (g_shm_fd < 0)
		return (cleanup_sem(), false);

	if (ftruncate(g_shm_fd, (off_t)sizeof(ShmLidarScanData)) < 0)
		return (cleanup_sem(), false);

	void *p = mmap(NULL, sizeof(ShmLidarScanData), PROT_READ | PROT_WRITE,
				   MAP_SHARED, g_shm_fd, 0);
	if (p == MAP_FAILED)
		return (cleanup_sem(), false);

	g_shm = (ShmLidarScanData *)p;

	g_sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
	if (g_sem == SEM_FAILED)
		return (cleanup_sem(), false);

	sem_wait(g_sem);
	g_shm->seq = NOUPDATED;
	for (int i = 0; i < 181; ++i)
		g_shm->distance_mm[i] = 0;
	sem_post(g_sem);

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
	sem_wait(g_sem);
	for (int i = -90; i <= 90; ++i)
		g_shm->distance_mm[i + 90] = scan_data.getDistance(i);
	g_shm->seq = UPDATED;
	sem_post(g_sem);
}

int main() {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	if (!start_lidar())
		return (1);
	if (!init_sem())
		return (1);

	LidarScanData scan_data;
	while (!g_stop) {
		if (receiveScanData(&scan_data))
			write_to_mem(scan_data);
		else
			usleep(1000);
	}

	// 終了処理
	if (g_lidar) {
		g_lidar->stop();
		g_lidar->disconnect();
		delete g_lidar;
	}
	if (g_shm)
		munmap(g_shm, sizeof(ShmLidarScanData));
	if (g_shm_fd >= 0)
		close(g_shm_fd);
	if (g_sem != SEM_FAILED)
		sem_close(g_sem);
	shm_unlink(SHM_NAME);
	sem_unlink(SEM_NAME);
	return (0);
}
