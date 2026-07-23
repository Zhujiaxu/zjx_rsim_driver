#pragma once

#include "PlanningStartPoint.hpp"

#include <cstddef>
#include <vector>

namespace rsim_driver::plugin_internal
{

bool IsValidTrajectoryPoint(const PlanningTrajectoryPoint &point);

bool FindTrajectoryPointAtTime(
    const std::vector<PlanningTrajectoryPoint> &trajectory,
    double queryTime,
    PlanningTrajectoryPoint *point,
    std::size_t *targetIndex);

} // namespace rsim_driver::plugin_internal
