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
    struct EgoBox
    {
        SlPoint left_front;
        SlPoint right_front;
        SlPoint left_rear;
        SlPoint right_rear;
    };
    struct ObsBox
    {
        SlPoint left_front;
        SlPoint right_front;
        SlPoint left_rear;
        SlPoint right_rear;
    };
    inline EgoBox ComputeEgoBox(const SlPoint &egoCenter, double egoLength, double egoWidth)
    {
        EgoBox egoBox;
        const double halfLength = 0.5 * std::max(0.0, egoLength);
        const double halfWidth = 0.5 * std::max(0.0, egoWidth);

        egoBox.left_front.s = egoCenter.s + halfLength;
        egoBox.left_front.l = egoCenter.l + halfWidth;

        egoBox.right_front.s = egoCenter.s + halfLength;
        egoBox.right_front.l = egoCenter.l - halfWidth;

        egoBox.left_rear.s = egoCenter.s - halfLength;
        egoBox.left_rear.l = egoCenter.l + halfWidth;

        egoBox.right_rear.s = egoCenter.s - halfLength;
        egoBox.right_rear.l = egoCenter.l - halfWidth;

        return egoBox;
    } 
    inline ObsBox ComputeObsBox(const SlPoint &obsCenter, double obsLength, double obsWidth)
    {
        ObsBox obsBox;
        const double halfLength = 0.5 * std::max(0.0, obsLength);
        const double halfWidth = 0.5 * std::max(0.0, obsWidth);

        obsBox.left_front.s = obsCenter.s + halfLength;
        obsBox.left_front.l = obsCenter.l + halfWidth;

        obsBox.right_front.s = obsCenter.s + halfLength;
        obsBox.right_front.l = obsCenter.l - halfWidth;

        obsBox.left_rear.s = obsCenter.s - halfLength;
        obsBox.left_rear.l = obsCenter.l + halfWidth;

        obsBox.right_rear.s = obsCenter.s - halfLength;
        obsBox.right_rear.l = obsCenter.l - halfWidth;

        return obsBox;
    }

    struct StaticCollisionCostConfig
    {
        double collision_distance = 0.6;
        double risk_distance = 2.5;
        double infinity_cost = std::numeric_limits<double>::infinity();
        double ego_length = 4.0;
        double ego_width = 2.0;
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
            EgoBox egoBox = ComputeEgoBox(point, config.ego_length, config.ego_width);
            ObsBox obsBox = ComputeObsBox({obstacle.s, obstacle.l}, obstacle.length, obstacle.width);
            

            
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
