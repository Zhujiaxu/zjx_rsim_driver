/*
 * CollisionChecker — shared collision detection used by planners.
 * Free functions parameterized with explicit vehicle dimensions.
 */
#pragma once

#include <algorithm>
#include <cmath>

#include "DriverTypes.hpp"

namespace rsim_driver
{

// Check whether two axis-aligned boxes overlap in Frenet s-d space.
inline bool FrenetBoxesOverlap(double s0, double d0, double length0, double width0,
                               double s1, double d1, double length1, double width1,
                               double marginLong, double marginLat)
{
    double longLimit = (length0 + length1) * 0.5 + marginLong;
    if (std::fabs(s0 - s1) > longLimit)
        return false;
    double latLimit = (width0 + width1) * 0.5 + marginLat;
    return std::fabs(d0 - d1) <= latLimit;
}

// Collision check of a planned trajectory against obstacle list.
// Obstacles are predicted with constant velocity: obs_s(t) = obs.s + obs.speed * t.
inline bool HasCollision(const PlannedTrajectory& trajectory,
                         const Obstacle* obstacles,
                         int numObs,
                         double egoLength,
                         double egoWidth,
                         double collisionLongBuffer,
                         double safeMarginLat)
{
    if (obstacles == nullptr || numObs <= 0)
        return false;
    for (int i = 0; i < trajectory.numPoints; ++i)
    {
        const TrajectoryPoint& p = trajectory.points[i];
        for (int j = 0; j < numObs; ++j)
        {
            const Obstacle& obs = obstacles[j];
            const double obsS = obs.s + obs.speed * p.t;
            if (FrenetBoxesOverlap(p.s, p.d, egoLength, egoWidth,
                                   obsS, obs.d, obs.length, obs.width,
                                   collisionLongBuffer, safeMarginLat))
                return true;
        }
    }
    return false;
}

// Minimum clearance between trajectory and obstacles (axis-aligned Frenet metric).
// Returns max(0, min(per-axis clearance)).
inline double MinObstacleClearance(const PlannedTrajectory& trajectory,
                                   const Obstacle* obstacles,
                                   int numObs,
                                   double egoLength,
                                   double egoWidth)
{
    if (obstacles == nullptr || numObs <= 0)
        return LARGE_NUMBER;
    double best = LARGE_NUMBER;
    for (int i = 0; i < trajectory.numPoints; ++i)
    {
        const TrajectoryPoint& p = trajectory.points[i];
        for (int j = 0; j < numObs; ++j)
        {
            const Obstacle& obs = obstacles[j];
            const double obsS = obs.s + obs.speed * p.t;
            double longClearance = std::fabs(p.s - obsS)
                                 - 0.5 * (egoLength + obs.length);
            double latClearance = std::fabs(p.d - obs.d)
                                - 0.5 * (egoWidth + obs.width);
            best = std::min(best, std::max(0.0, std::min(longClearance, latClearance)));
        }
    }
    return best;
}

}  // namespace rsim_driver
