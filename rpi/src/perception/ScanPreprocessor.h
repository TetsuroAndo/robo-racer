#pragma once

#include "config/Params.h"
#include "planning/planning_types.h"
#include "perception/LidarReceiver.h"

#include <vector>

namespace perception {

class ScanPreprocessor {
public:
	planning::Scan process(const std::vector< LidarData > &points,
						   const cfg::Params &params) const;
};

} // namespace perception
