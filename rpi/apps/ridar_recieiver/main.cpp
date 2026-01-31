#include "../../src/config/Config.h"
#include "LidarScanData.hpp"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <csignal>
#include <iostream>
#include <semaphore.h>
#include <unistd.h>

static sl::ILidarDriver *g_lidar = 0;
static sl::IChannel *g_ch = 0;

static int g_shm_fd = -1;
static ShmLidarScanData *g_shm = 0;
static sem_t *g_sem = SEM_FAILED;

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int sig) {
	// std::cout << "\nSignal " << sig << " received, shutting down..." <<
	// std::endl;
	g_stop = 1;
}

bool start_ridar() {
	const char *dev = DEFAULT_LIDAR_DEVICE;
	int baud = DEFAULT_LIDAR_BAUD;

	g_ch = *sl::createSerialPortChannel(dev, baud);
	if (!g_ch)
		return (false);

	g_lidar = *sl::createLidarDriver();
	if (!g_lidar)
		return (false);

	if (SL_IS_FAIL(g_lidar->connect(g_ch)))
		return (false);

	sl_lidar_response_device_info_t info;
	if (SL_IS_FAIL(g_lidar->getDeviceInfo(info)))
		return (false);

	sl_lidar_response_device_health_t health;
	if (SL_IS_FAIL(g_lidar->getHealth(health)))
		return (false);

	if (SL_IS_FAIL(g_lidar->setMotorSpeed()))
		return (false);

	if (SL_IS_FAIL(g_lidar->startScan(false, true)))
		return (false);
	return (true);
}

bool init_sem() {}

static bool receiveScanData() {
	sl_lidar_response_measurement_node_hq_t nodes[cfg::LIDAR_NODE_MAX];
	size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);

	sl_result ans = g_lidar->grabScanDataHq(nodes, nodeCount);
	if (!SL_IS_OK(ans) || nodeCount == 0)
		return (false);

	// 上記の取得だけではソートされていないため以下の関数でソートする
	ans = g_lidar->ascendScanData(nodes, nodeCount);
	if (!SL_IS_OK(ans))
		return (false);

	// 四捨五入した角度を格納する。値が無い空ビンは0をセットする
	// 同じビンに複数値の候補がある場合は距離が近いほうを採用する

	// for (int i = 0; i < 180; ++i)
	// 	out_mm[i] = -1;

	// for (size_t i = 0; i < nodeCount; i++)
	// {
	// 	if (nodes[i].dist_mm_q2 == 0)
	// 		continue;

	// 	float angle = (float)nodes[i].angle_z_q14 * 90.f / 16384.f;
	// 	angle = normalize_deg_180(angle);

	// 	int32_t dist = (int32_t)(nodes[i].dist_mm_q2 >> 2);
	// 	if (dist <= 0)
	// 		continue;

	// 	int idx;
	// 	if (!angle_to_bin(angle, idx))
	// 		continue;

	// 	// 同一ビンは最小距離
	// 	if (out_mm[idx] < 0 || dist < out_mm[idx])
	// 		out_mm[idx] = dist;
	// }
}

int main() {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	if (start_ridar())
		return (1);
	if (init_sem())
		return (1);
	while (!g_stop) {
		receiveScanData();
		write_to_mem();
	}

	// 終了処理
	// int fd = shm_open("/lidar_scan", O_CREAT | O_RDWR, 0666);
}

// // ===== stop flag =====
// static volatile sig_atomic_t g_stop = 0;
// static void on_sig(int) { g_stop = 1; }

// // ===== shared memory layout (POD) =====
// static const char* SHM_NAME = "/lidar_scan";
// static const char* SEM_NAME = "/lidar_scan_sem";

// struct ShmLidarScanData {
//     uint32_t seq;
//     int32_t  distance_mm[180]; // [-90, +90)
// };

// // ===== global handles (最小) =====
// static sl::ILidarDriver* g_lidar = 0;
// static sl::IChannel*     g_ch    = 0;

// static int               g_shm_fd = -1;
// static ShmLidarScanData* g_shm    = 0;
// static sem_t*            g_sem    = SEM_FAILED;

// // ===== util =====
// static inline float normalize_deg_180(float deg) {
//     while (deg >= 180.f) deg -= 360.f;
//     while (deg <  -180.f) deg += 360.f;
//     return deg;
// }

// static inline bool angle_to_bin(float angle_deg, int& idx_out) {
//     // [-90, +90) を 180ビン
//     int a = (int)angle_deg; // trunc。floorにしたければ調整
//     if (a < -90 || a >= 90) return false;
//     idx_out = a + 90; // 0..179
//     return true;
// }

// // ===== 1) LiDAR start =====
// static bool start_lidar()
// {
//     const char* dev  = DEFAULT_LIDAR_DEVICE;
//     int         baud = DEFAULT_LIDAR_BAUD;

//     g_ch = *sl::createSerialPortChannel(dev, baud);
//     if (!g_ch) return false;

//     g_lidar = *sl::createLidarDriver();
//     if (!g_lidar) return false;

//     if (SL_IS_FAIL(g_lidar->connect(g_ch))) return false;

//     sl_lidar_response_device_info_t info;
//     if (SL_IS_FAIL(g_lidar->getDeviceInfo(info))) return false;

//     sl_lidar_response_device_health_t health;
//     if (SL_IS_FAIL(g_lidar->getHealth(health))) return false;

//     // モータ制御（SDKによってシグネチャが違う場合あり）
//     // もしコンパイルが通らなければ、この行だけエラー内容に合わせて直す。
//     g_lidar->setMotorSpeed(600);

//     if (SL_IS_FAIL(g_lidar->startScan(false, true))) return false;

//     return true;
// }

// // ===== 2) IPC init =====
// static bool init_ipc()
// {
//     g_shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
//     if (g_shm_fd < 0) return false;

//     if (ftruncate(g_shm_fd, (off_t)sizeof(ShmLidarScanData)) < 0) return
//     false;

//     void* p = mmap(NULL, sizeof(ShmLidarScanData),
//                    PROT_READ | PROT_WRITE, MAP_SHARED,
//                    g_shm_fd, 0);
//     if (p == MAP_FAILED) return false;

//     g_shm = (ShmLidarScanData*)p;

//     g_sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
//     if (g_sem == SEM_FAILED) return false;

//     // 初期化（サーバのみ）
//     sem_wait(g_sem);
//     g_shm->seq = 0;
//     for (int i = 0; i < 180; ++i) g_shm->distance_mm[i] = -1;
//     sem_post(g_sem);

//     return true;
// }

// // ===== 3) Grab + bin into local[180] =====
// static bool receiveScanData_binned(int32_t out_mm[180])
// {
//     sl_lidar_response_measurement_node_hq_t nodes[cfg::LIDAR_NODE_MAX];
//     size_t nodeCount = sizeof(nodes) / sizeof(nodes[0]);

//     sl_result ans = g_lidar->grabScanDataHq(nodes, nodeCount, 1000);
//     if (!SL_IS_OK(ans) || nodeCount == 0) return false;

//     ans = g_lidar->ascendScanData(nodes, nodeCount);
//     if (!SL_IS_OK(ans)) return false;

//     for (int i = 0; i < 180; ++i) out_mm[i] = -1;

//     for (size_t i = 0; i < nodeCount; i++) {
//         if (nodes[i].dist_mm_q2 == 0) continue;

//         float angle = (float)nodes[i].angle_z_q14 * 90.f / 16384.f;
//         angle = normalize_deg_180(angle);

//         int32_t dist = (int32_t)(nodes[i].dist_mm_q2 >> 2);
//         if (dist <= 0) continue;

//         int idx;
//         if (!angle_to_bin(angle, idx)) continue;

//         // 同一ビンは最小距離
//         if (out_mm[idx] < 0 || dist < out_mm[idx]) out_mm[idx] = dist;
//     }
//     return true;
// }

// // ===== 4) publish to shared memory =====
// static void publishScan(const int32_t mm[180])
// {
//     sem_wait(g_sem);
//     for (int i = 0; i < 180; ++i) g_shm->distance_mm[i] = mm[i];
//     g_shm->seq++;
//     sem_post(g_sem);
// }

// int main()
// {
//     signal(SIGINT,  on_sig);
//     signal(SIGTERM, on_sig);

//     if (!start_lidar()) return 1;
//     if (!init_ipc())    return 1;

//     int32_t local[180];

//     while (!g_stop) {
//         if (receiveScanData_binned(local)) {
//             publishScan(local);
//         } else {
//             usleep(10 * 1000);
//         }
//     }

//     // 終了処理（最小）
//     if (g_lidar) {
//         g_lidar->stop();
//         g_lidar->disconnect();
//         sl::disposeDriver(g_lidar);
//     }

//     if (g_shm) munmap(g_shm, sizeof(ShmLidarScanData));
//     if (g_shm_fd >= 0) close(g_shm_fd);
//     if (g_sem != SEM_FAILED) sem_close(g_sem);

//     shm_unlink(SHM_NAME);
//     sem_unlink(SEM_NAME);
//     return 0;
// }
