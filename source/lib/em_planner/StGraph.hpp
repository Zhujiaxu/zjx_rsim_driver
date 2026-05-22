/*
 * StGraph — builds ST-space obstacle boundaries for speed planning.
 *
 * Projects obstacles along a smoothed path (QP path) into (t, s) space.
 * Each StBoundary is the s-interval occupied by an obstacle at a given time.
 */
#pragma once

#include "DriverTypes.hpp"
#include "IPredictor.hpp"

#include <vector>

namespace rsim_driver
{

struct StBoundary
{
    double t    = 0.0;   // time of this boundary slice
    double sMin = 0.0;   // lower s of occupied region
    double sMax = 0.0;   // upper s of occupied region
    int    obsIndex = -1; // source obstacle index
};

// A simplified point on the smoothed path (used within speed planning).
struct SpeedPathPoint
{
    double s = 0.0;  // arc length along reference line
    double x = 0.0;  // world x
    double y = 0.0;  // world y
};

class StGraph
{
public:
    // Build ST boundaries from obstacles projected along the given path.
    // path: sequence of (s, x, y) along the smoothed QP path.
    // Each obstacle is projected with constant velocity.
    // Returns boundaries at every dt step from 0 to horizonTime.
    static std::vector<StBoundary> Build(
        const std::vector<SpeedPathPoint>& path,
        const Obstacle* obstacles,
        int numObstacles,
        double horizonTime,
        double dt,
        double egoLength,
        double obsLengthDefault);
};

}  // namespace rsim_driver
