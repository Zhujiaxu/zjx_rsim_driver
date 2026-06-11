#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rsim_driver
{

struct SlPoint
{
    double s = 0.0;
    double l = 0.0;
};

struct CollisionCostConfig
{
    double collision_distance = 1.6;
    double risk_distance = 3.0;
    double infinity_cost = std::numeric_limits<double>::infinity();
};

inline double ObstacleCollisionCost(const SlPoint& point,
                                    const std::vector<SlPoint>& obstacles,
                                    const CollisionCostConfig& config = {})
{
    const double collisionDistance = std::max(0.0, config.collision_distance);
    const double riskDistance = std::max(collisionDistance, config.risk_distance);
    double totalCost = 0.0;

    for (const SlPoint& obstacle : obstacles)
    {
        const double ds = point.s - obstacle.s;
        const double dl = point.l - obstacle.l;
        const double distance = std::sqrt(ds * ds + dl * dl);

        if (distance <= collisionDistance)
            return config.infinity_cost;
        if (distance >= riskDistance)
            continue;
        if (riskDistance <= collisionDistance)
            continue;

        totalCost += (riskDistance - distance) / (riskDistance - collisionDistance);
    }

    return totalCost;
}

}  // namespace rsim_driver
