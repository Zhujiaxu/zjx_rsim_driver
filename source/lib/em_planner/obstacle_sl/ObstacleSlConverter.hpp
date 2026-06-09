#pragma once

#include "CartesianToFrenet.hpp"
#include "obstacle_sl/SlObstacle.hpp"
#include "perception/StaticObstacleDetector.hpp"

#include <utility>
#include <vector>

namespace rsim_driver
{

namespace obstacle_sl_detail
{

struct ObstacleCartesianPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
};

}  // namespace obstacle_sl_detail

template <typename RefPointT>
bool ConvertStaticObstaclesToSl(
    const std::vector<StaticObstacle>& staticObstacles,
    const std::vector<RefPointT>& referencePoints,
    std::vector<SlObstacle>* slObstacles)
{
    if (slObstacles == nullptr || referencePoints.empty())
        return false;

    std::vector<SlObstacle> converted;
    converted.reserve(staticObstacles.size());

    for (const StaticObstacle& obstacle : staticObstacles)
    {
        obstacle_sl_detail::ObstacleCartesianPoint point;
        point.x = obstacle.x;
        point.y = obstacle.y;
        point.heading = obstacle.heading;

        CartesianFrenetState frenet;
        if (!cartesian_to_frenet_detail::CartesianPointToFrenet(
                referencePoints, point, 0.0, &frenet))
            return false;

        SlObstacle slObstacle;
        slObstacle.id = obstacle.id;
        slObstacle.s = frenet.s;
        slObstacle.l = frenet.l;
        converted.push_back(slObstacle);
    }

    *slObstacles = std::move(converted);
    return true;
}

}  // namespace rsim_driver
