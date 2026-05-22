/*
 * QpSpeedOptimizer — QP-based speed smoothing using OSQP.
 *
 * Takes the coarse DP speed profile and refines it with a quadratic
 * smoothness + tracking cost, respecting ST-boundary position constraints
 * and speed limits.
 */
#pragma once

#include "DriverTypes.hpp"
#include "DpSpeedOptimizer.hpp"
#include "StGraph.hpp"
#include <vector>

namespace rsim_driver
{

class QpSpeedOptimizer
{
public:
    struct Config
    {
        double wRefV   = 1.0;    // speed tracking
        double wAccel  = 10.0;   // acceleration smoothness
        double wJerk   = 100.0;  // jerk smoothness
        double wEndV   = 1.0;    // terminal speed
        double wEndS   = 1.0;    // terminal position
        int    maxIter = 4000;
    };

    Config config;

    bool Optimize(const FrenetState& ego,
                  const std::vector<DpSpeedPoint>& dpSpeed,
                  const std::vector<StBoundary>& stBoundaries,
                  double speedLimit,
                  double maxAccel,
                  double maxDecel,
                  std::vector<QpSpeedPoint>* qpSpeed) const;
};

}  // namespace rsim_driver
