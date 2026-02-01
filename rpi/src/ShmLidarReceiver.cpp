#include "ShmLidarReceiver.h"

#include "config/Config.h"

#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
static constexpr const char *kShmName = "/lidar_scan";
static constexpr const char *kSemName = "/lidar_scan_sem";
static constexpr uint32_t kSeqNoUpdate = 0;
} // namespace

ShmLidarReceiver::ShmLidarReceiver()
	: _shmFd(-1), _shm(nullptr), _sem(SEM_FAILED), _lastSeq(0),
	  _connected(false) {}

ShmLidarReceiver::~ShmLidarReceiver() { cleanup(); }

bool ShmLidarReceiver::connect() {
	if (_connected)
		return true;

	_shmFd = shm_open(kShmName, O_RDONLY, 0);
	if (_shmFd < 0) {
		cleanup();
		return false;
	}

	void *p =
		mmap(NULL, sizeof(ShmLidarScanData), PROT_READ, MAP_SHARED, _shmFd, 0);
	if (p == MAP_FAILED) {
		cleanup();
		return false;
	}
	_shm = static_cast< ShmLidarScanData * >(p);

	_sem = sem_open(kSemName, 0);
	if (_sem == SEM_FAILED) {
		cleanup();
		return false;
	}

	_lastSeq = 0;
	_connected = true;
	return true;
}

bool ShmLidarReceiver::isConnected() const { return _connected; }

bool ShmLidarReceiver::getLatestData(std::vector< LidarData > &out) {
	if (!_connected)
		return false;

	if (sem_trywait(_sem) == -1) {
		if (errno == EAGAIN || errno == EINTR)
			return false;
		return false;
	}

	uint32_t seq = _shm->seq;
	if (seq == kSeqNoUpdate || seq == _lastSeq) {
		sem_post(_sem);
		return false;
	}

	std::array< int32_t, 181 > distances;
	for (size_t i = 0; i < distances.size(); ++i)
		distances[i] = _shm->distance_mm[i];

	sem_post(_sem);

	_lastSeq = seq;

	out.clear();
	out.reserve(distances.size());
	for (int angle = -90; angle <= 90; ++angle) {
		int32_t dist = distances[static_cast< size_t >(angle + 90)];
		if (dist < static_cast< int32_t >(cfg::LIDAR_DIST_MIN_MM))
			continue;
		out.emplace_back(dist, static_cast< float >(angle));
	}

	return !out.empty();
}

void ShmLidarReceiver::cleanup() {
	if (_sem != SEM_FAILED) {
		sem_close(_sem);
		_sem = SEM_FAILED;
	}

	if (_shm && _shm != MAP_FAILED) {
		munmap(_shm, sizeof(ShmLidarScanData));
		_shm = nullptr;
	}

	if (_shmFd >= 0) {
		close(_shmFd);
		_shmFd = -1;
	}

	_connected = false;
}
