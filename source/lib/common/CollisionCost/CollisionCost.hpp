#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "PointToBoundaryDistance.hpp"

namespace rsim_driver
{

    struct SlPoint
    {
        double s = 0.0;
        double l = 0.0;
    };

    struct StaticCollisionCostConfig
    {
        double collision_distance = 2.0;
        double risk_distance = 5.0;
        double infinity_cost = std::numeric_limits<double>::infinity();
    };

    struct DynamicCollisionCostConfig
    {
        double collision_distance = 4.0;
        double risk_distance = 8.0;
        double infinity_cost = std::numeric_limits<double>::infinity();
    };

    inline double DistanceCollisionCost(double distance,
                                        const StaticCollisionCostConfig &config)
    {
        const double collisionDistance = std::max(0.0, config.collision_distance);
        const double riskDistance = std::max(collisionDistance, config.risk_distance);

        if (distance <= collisionDistance)
            return config.infinity_cost;
        if (distance >= riskDistance || riskDistance <= collisionDistance)
            return 0.0;

        return (riskDistance - distance) / (riskDistance - collisionDistance);
    }

    inline double StaticObstacleCollisionCost(const SlPoint &point,
                                              const std::vector<SlPoint> &staticobstacles,
                                              const StaticCollisionCostConfig &config = {})
    {
        double totalCost = 0.0;

        for (const SlPoint &obstacle : staticobstacles)
        {
            const double ds = point.s - obstacle.s;
            const double dl = point.l - obstacle.l;
            const double distance = std::sqrt(ds * ds + dl * dl);
            const double cost = DistanceCollisionCost(distance, config);

            if (!std::isfinite(cost) || cost == config.infinity_cost)
                return config.infinity_cost;
            totalCost += cost;
        }

        return totalCost;
    }

    template <typename StaticObstacleT>
    inline double StaticObstacleCollisionCost(
        const SlPoint &point,
        const std::vector<StaticObstacleT> &staticobstacles,
        const StaticCollisionCostConfig &config = {})
    {
        double totalCost = 0.0;

        for (const StaticObstacleT &obstacle : staticobstacles)
        {
            if (!std::isfinite(obstacle.s) ||
                !std::isfinite(obstacle.l) ||
                !std::isfinite(obstacle.length) ||
                !std::isfinite(obstacle.width))
            {
                continue;
            }

            const double halfLength = 0.5 * std::max(0.0, obstacle.length);
            const double halfWidth = 0.5 * std::max(0.0, obstacle.width);
            const double ds =
                std::max(0.0, std::fabs(point.s - obstacle.s) - halfLength);
            const double dl =
                std::max(0.0, std::fabs(point.l - obstacle.l) - halfWidth);
            const double distance = std::hypot(ds, dl);
            const double cost = DistanceCollisionCost(distance, config);

            if (!std::isfinite(cost) || cost == config.infinity_cost)
                return config.infinity_cost;
            totalCost += cost;
        }

        return totalCost;
    }
    template <typename CutInAndOutInfoT>
    inline double DynamicObstacleCollisionCost(const StPoint &point,
                                              const std::vector<CutInAndOutInfoT> &cutInAndOutInfos,
                                              const DynamicCollisionCostConfig &config = {})
    {
        const double collisionDistance = std::max(0.0, config.collision_distance);
        const double riskDistance = std::max(collisionDistance, config.risk_distance);
        double totalCost = 0.0;

        std::vector<PointToBoundaryDistanceResult> distances;
        if (!ComputePointToBoundaryDistances(cutInAndOutInfos,
                                                    point,
                                                    &distances))
        {
            return config.infinity_cost;
        }

        for (const PointToBoundaryDistanceResult &distanceResult : distances)
        {
            const double distance = distanceResult.distance;

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

} // namespace rsim_driver
