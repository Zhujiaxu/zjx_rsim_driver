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

    inline double StaticObstacleCollisionCost(const SlPoint &point,
                                              const std::vector<SlPoint> &staticobstacles,
                                              const StaticCollisionCostConfig &config = {})
    {
        const double collisionDistance = std::max(0.0, config.collision_distance);
        const double riskDistance = std::max(collisionDistance, config.risk_distance);
        double totalCost = 0.0;

        for (const SlPoint &obstacle : staticobstacles)
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
    template <typename CutInAndOutInfoT>
    inline double DynamicObstacleCollisionCost(const StPoint &point,
                                              const std::vector<CutInAndOutInfoT> &cutInAndOutInfos,
                                              const DynamicCollisionCostConfig &config = {})
    {
        const double collisionDistance = std::max(0.0, config.collision_distance);
        const double riskDistance = std::max(collisionDistance, config.risk_distance);
        double totalCost = 0.0;

        for (const CutInAndOutInfoT &cutInAndOutInfo : cutInAndOutInfos)
        {
            const StPoint cutInPoint{cutInAndOutInfo.sin, cutInAndOutInfo.tin};
            const StPoint cutOutPoint{cutInAndOutInfo.sout, cutInAndOutInfo.tout};
            const double distance = PointToLineSegmentDistance(point, cutInPoint, cutOutPoint);

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
