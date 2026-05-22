/*
 * DpSpeedOptimizer — DP-based coarse speed planning.
 *
 * Uses dynamic programming on a (t, s) grid with ST-boundary collision costs
 * to find the lowest-cost speed profile.
 */
#pragma once

#include "DriverTypes.hpp"
#include "StGraph.hpp"
#include <vector>

namespace rsim_driver
{

struct DpSpeedPoint
{
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
};

struct QpSpeedPoint
{
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
};

class DpSpeedOptimizer
{
public:
    struct Config
    {
        double tStep      = 0.5;   // time step (s)
        double sStep      = 2.0;   // s spacing (m)
        double wRefV      = 1.0;   // reference speed tracking
        double wAccel     = 10.0;  // acceleration cost
        double wJerk      = 100.0; // jerk cost
        double wCollision = 1000.0;// collision cost
    };

    Config config;

    // ego: current Frenet state
    // desiredSpeed: target cruise speed
    // speedLimit: max speed from curvature constraints
    // stBoundaries: ST obstacle projections
    // horizonTime: planning horizon (s)
    bool Optimize(const FrenetState& ego,
                  double desiredSpeed,
                  double speedLimit,
                  const std::vector<StBoundary>& stBoundaries,
                  double horizonTime,
                  std::vector<DpSpeedPoint>* dpSpeed) const;
};

}  // namespace rsim_driver
