#ifndef LIDARSCANDATA_HPP
# define LIDARSCANDATA_HPP

#include <cstdint>

struct ShmLidarScanData
{
	uint32_t	seq;
	int32_t		distance_mm[181];
};

class	LidarScanData
{
	private:
		int32_t	distance_mm[181];
	public:
		LidarScanData();
		~LidarScanData();

		int32_t	getDistance(int32_t angle) const;
		bool	setDistance(int32_t angle, int32_t value);
};

#endif
