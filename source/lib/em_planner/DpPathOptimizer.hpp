/*
 * DpPathOptimizer — DP-based coarse path planning.
 *
 * Samples lateral offsets at longitudinal steps along the reference line
 * and uses dynamic programming to find the lowest-cost path.
 */
#pragma once

#include "DriverTypes.hpp"
#include "IPredictor.hpp"
#include "ReferenceLine.hpp"

#include <vector>

namespace rsim_driver
{

struct DpPathPoint
{
    double s   = 0.0;  // arc length along reference line
    double l   = 0.0;  // lateral offset
    double dl  = 0.0;  // l derivative w.r.t. s (estimated from DP)
    double ddl = 0.0;  // l second derivative w.r.t. s
};

struct QpPathPoint
{
    double s   = 0.0;  // arc length along reference line
    double l   = 0.0;  // lateral offset
    double dl  = 0.0;  // l derivative w.r.t. s
    double ddl = 0.0;  // l second derivative w.r.t. s
};

class DpPathOptimizer
{
public:
    struct Config
    {
        int    numLongitudinal = 7;     // number of s columns
        int    numLateral      = 5;     // lateral rows per column
        double sStep           = 5.0;   // longitudinal spacing (m)
        double lStep           = 0.5;   // lateral spacing (m)
        double wRefOffset      = 1.0;   // weight for deviation from lane centre
        double wSmooth         = 10.0;  // weight for smoothness (ddl)
        double wCollision      = 500.0; // weight for obstacle collision
        double collisionMargin = 0.5;   // extra lateral margin for collision check
    };

    Config config;

    // Run DP.  Returns false if no feasible path exists.
    bool Optimize(const ReferenceLine& refLine,
                  const FrenetState& ego,
                  const Obstacle* obstacles,
                  int numObstacles,
                  const IPredictor& predictor,
                  std::vector<DpPathPoint>* dpPath) const;
};

}  // namespace rsim_driver
