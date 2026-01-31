#pragma once

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

struct LidarData {
	int distance;
	float angle;

	LidarData(int distance, float angle) : distance(distance), angle(angle) {}
};

class LidarReceiver {
public:
	LidarReceiver(const char *lidar_dev_c, int lidar_baud);
	~LidarReceiver();

	// 従来のインターフェース（互換性用）
	std::vector< LidarData > receive();

	// スレッド管理
	void startReceivingThread();
	void stopReceivingThread();
	bool getLatestData(std::vector< LidarData > &out);

protected:
	void _receivingThreadLoop();

	sl::IChannel *_channel;
	sl::ILidarDriver *_lidar;

	// スレッド関連
	std::thread _receivingThread;
	std::mutex _dataMutex;
	std::condition_variable _dataCV;
	std::queue< std::vector< LidarData > > _dataQueue;
	bool _isReceiving;

	std::vector< LidarData > _receiveOnce();

private:
	void _init(const char *lidar_dev_c, int lidar_baud);
};
