/*
 * PathTimeTrajectory — merges QP path and QP speed profiles into a trajectory.
 */
#pragma once

#include "DriverTypes.hpp"
#include "DpPathOptimizer.hpp"
#include "DpSpeedOptimizer.hpp"
#include <vector>

namespace rsim_driver
{

class PathTimeTrajectory
{
public:
    // Merge QP path and speed profiles into FrenetTrajectory + PlannedTrajectory.
    // The FrenetTrajectory is a piecewise polynomial fitted to the profiles;
    // the PlannedTrajectory contains uniformly sampled trajectory points.
    static bool Combine(
        const std::vector<QpPathPoint>& qpPath,
        const std::vector<QpSpeedPoint>& qpSpeed,
        double simTime,
        FrenetTrajectory* frenet,
        PlannedTrajectory* trajectory);
};

}  // namespace rsim_driver
