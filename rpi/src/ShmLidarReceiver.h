#pragma once

#include "LidarReceiver.h"

#include <cstdint>
#include <semaphore.h>
#include <vector>

struct ShmLidarScanData {
	uint32_t seq;
	int32_t distance_mm[181];
};

class ShmLidarReceiver {
public:
	ShmLidarReceiver();
	~ShmLidarReceiver();

	bool connect();
	bool isConnected() const;
	bool getLatestData(std::vector< LidarData > &out);
	uint32_t lastSeq() const { return _lastSeq; }

private:
	void cleanup();

	int _shmFd;
	ShmLidarScanData *_shm;
	sem_t *_sem;
	uint32_t _lastSeq;
	bool _connected;
};
