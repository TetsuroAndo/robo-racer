#pragma once

#include "planning/IPlanner.h"

#include <vector>

namespace planning {

class PlannerFTG : public IPlanner {
public:
	PlanOutput plan(const Scan &scan, const cfg::Params &params) override;

private:
	int computeFrontMin(const Scan &scan, const cfg::Params &params) const;
	std::vector< int > inflateObstacles(const Scan &scan,
						 const cfg::Params &params) const;
	std::vector< GapDescriptor > findGaps(const Scan &scan,
					 const cfg::Params &params,
					 const std::vector< int > &inflated) const;
	size_t selectBestGap(const std::vector< GapDescriptor > &gaps) const;
	void applySafetyBubble(const Scan &scan, const cfg::Params &params,
						   std::vector< int > &inflated) const;
	void extendDisparities(const cfg::Params &params,
						   std::vector< int > &inflated) const;
	void clampGapAimPoint(const Scan &scan, const cfg::Params &params,
						  GapDescriptor &gap) const;
};

} // namespace planning
