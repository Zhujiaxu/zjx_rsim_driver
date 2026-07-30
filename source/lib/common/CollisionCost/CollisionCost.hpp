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

    struct SlAxisAlignedBox
    {
        double min_s = 0.0;
        double max_s = 0.0;
        double min_l = 0.0;
        double max_l = 0.0;
    };

    inline bool BuildSlAxisAlignedBox(const SlPoint &center,
                                      double length,
                                      double width,
                                      SlAxisAlignedBox *box)
    {
        if (box == nullptr ||
            !std::isfinite(center.s) || !std::isfinite(center.l) ||
            !std::isfinite(length) || length <= 0.0 ||
            !std::isfinite(width) || width <= 0.0)
        {
            return false;
        }

        const double halfLength = 0.5 * length;
        const double halfWidth = 0.5 * width;
        box->min_s = center.s - halfLength;
        box->max_s = center.s + halfLength;
        box->min_l = center.l - halfWidth;
        box->max_l = center.l + halfWidth;
        return true;
    }

    inline bool SlBoxesOverlap(const SlAxisAlignedBox &first,
                               const SlAxisAlignedBox &second)
    {
        constexpr double kGeometryEpsilon = 1e-9;
        return first.min_s <= second.max_s + kGeometryEpsilon &&
               second.min_s <= first.max_s + kGeometryEpsilon &&
               first.min_l <= second.max_l + kGeometryEpsilon &&
               second.min_l <= first.max_l + kGeometryEpsilon;
    }

    inline double SlBoxClearanceDistance(const SlAxisAlignedBox &first,
                                         const SlAxisAlignedBox &second)
    {
        const double sGap = std::max(
            {0.0, first.min_s - second.max_s, second.min_s - first.max_s});
        const double lGap = std::max(
            {0.0, first.min_l - second.max_l, second.min_l - first.max_l});
        return std::hypot(sGap, lGap);
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

    inline bool ValidStaticCollisionCostConfig(
        const StaticCollisionCostConfig &config)
    {
        return std::isfinite(config.collision_distance) &&
               config.collision_distance >= 0.0 &&
               std::isfinite(config.risk_distance) &&
               config.risk_distance > config.collision_distance &&
               !std::isnan(config.infinity_cost) &&
               config.infinity_cost > 0.0 &&
               std::isfinite(config.ego_length) &&
               config.ego_length > 0.0 &&
               std::isfinite(config.ego_width) &&
               config.ego_width > 0.0;
    }

    inline double DistanceCollisionCost(double distance,
                                        const StaticCollisionCostConfig &config)
    {
        if (!ValidStaticCollisionCostConfig(config) ||
            !std::isfinite(distance) || distance < 0.0)
        {
            return std::numeric_limits<double>::infinity();
        }

        if (distance <= config.collision_distance)
            return config.infinity_cost;
        if (distance >= config.risk_distance)
            return 0.0;

        return (config.risk_distance - distance) /
               (config.risk_distance - config.collision_distance);
    }

    template <typename StaticObstacleT>
    inline double StaticObstacleCollisionCost(
        const SlPoint &point,
        const std::vector<StaticObstacleT> &staticobstacles,
        const StaticCollisionCostConfig &config = {})
    {
        if (!ValidStaticCollisionCostConfig(config))
            return std::numeric_limits<double>::infinity();

        SlAxisAlignedBox egoBox;
        if (!BuildSlAxisAlignedBox(point,
                                   config.ego_length,
                                   config.ego_width,
                                   &egoBox))
        {
            return config.infinity_cost;
        }

        double totalCost = 0.0;

        for (const StaticObstacleT &obstacle : staticobstacles)
        {
            if (!std::isfinite(obstacle.s) ||
                !std::isfinite(obstacle.l) ||
                !std::isfinite(obstacle.length) ||
                obstacle.length <= 0.0 ||
                !std::isfinite(obstacle.width) ||
                obstacle.width <= 0.0)
            {
                return config.infinity_cost;
            }

            SlAxisAlignedBox obstacleBox;
            if (!BuildSlAxisAlignedBox({obstacle.s, obstacle.l},
                                       obstacle.length,
                                       obstacle.width,
                                       &obstacleBox))
            {
                return config.infinity_cost;
            }

            if (SlBoxesOverlap(egoBox, obstacleBox))
                return config.infinity_cost;

            const double distance =
                SlBoxClearanceDistance(egoBox, obstacleBox);
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
