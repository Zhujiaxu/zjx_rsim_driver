/*
 * StGraph implementation.
 */
#include "StGraph.hpp"
#include <algorithm>
#include <cmath>

namespace rsim_driver
{

std::vector<StBoundary> StGraph::Build(
    const std::vector<SpeedPathPoint>& /*path*/,
    const Obstacle* obstacles,
    int numObstacles,
    double horizonTime,
    double dt,
    double egoLength,
    double obsLengthDefault)
{
    std::vector<StBoundary> boundaries;
    if (obstacles == nullptr || numObstacles <= 0)
        return boundaries;

    if (dt <= 1e-6)
        dt = 0.5;

    int numSteps = static_cast<int>(std::ceil(horizonTime / dt)) + 1;

    for (int step = 0; step < numSteps; ++step)
    {
        double t = step * dt;
        if (t > horizonTime + 1e-6)
            t = horizonTime;

        for (int oi = 0; oi < numObstacles; ++oi)
        {
            const Obstacle& obs = obstacles[oi];
            double obsS = obs.s + obs.speed * t;

            double halfLen = 0.5 * (egoLength + (obs.length > 0.0 ? obs.length : obsLengthDefault));

            StBoundary b;
            b.t        = t;
            b.sMin     = obsS - halfLen;
            b.sMax     = obsS + halfLen;
            b.obsIndex = oi;
            boundaries.push_back(b);
        }
    }

    return boundaries;
}

}  // namespace rsim_driver
