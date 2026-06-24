#pragma once

#include "CartesianToFrenet.hpp"
#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <utility>
#include <vector>

namespace rsim_driver
{

struct StaticFrenetObstacle
{
    int32_t id = 0;
    double s = 0.0;
    double l = 0.0;
    double length = 0.0;
    double width = 0.0;
};
using DynamicFrenetState = CartesianFrenetState;
struct DynamicFrenetObstacle
{
    int32_t id = 0;
    DynamicFrenetState dynamicfrenetstate;
};

struct StaticFrenetObstaclePerceptionResult
{
    std::vector<StaticFrenetObstacle> staticobstacles;
};

struct DynamicFrenetObstaclePerceptionResult
{
    std::vector<DynamicFrenetObstacle> dynamicobstacles;
};

struct FrenetObstaclePerceptionConfig
{
    double static_speed_threshold = 0.1;
};

namespace frenet_obstacle_perception_detail
{

struct ObstacleCartesianPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
};

inline double ActorPlanarSpeed(const rsim_plugin::ActorState& actor)
{
    const double velocitySpeed =
        std::sqrt(actor.vel_x * actor.vel_x + actor.vel_y * actor.vel_y);
    return std::max(std::fabs(actor.speed), velocitySpeed);
}

inline double ActorLongitudinalAccel(const rsim_plugin::ActorState& actor)
{
    return std::sqrt(actor.acc_x * actor.acc_x + actor.acc_y * actor.acc_y);
}

template <typename RefPointT>
StaticFrenetObstacle ToStaticFrenetObstacle(
    const rsim_plugin::ActorState& actor,
    const std::vector<RefPointT>& referencePoints)
{
    const std::size_t matchIndex =
        FindMatchPointIndex(referencePoints, actor.x, actor.y);
    const RefPointT& matchedPoint = referencePoints[matchIndex];
    const RefPointT projectionPoint =
        FindProjectionPoint(referencePoints, actor.x, actor.y);

    const double tangentX = std::cos(matchedPoint.hdg);
    const double tangentY = std::sin(matchedPoint.hdg);
    const double normalX = -std::sin(matchedPoint.hdg);
    const double normalY = std::cos(matchedPoint.hdg);
    const double projectionDx = projectionPoint.x - matchedPoint.x;
    const double projectionDy = projectionPoint.y - matchedPoint.y;
    const double lateralDx = actor.x - projectionPoint.x;
    const double lateralDy = actor.y - projectionPoint.y;

    StaticFrenetObstacle obstacle;
    obstacle.id = actor.id;
    obstacle.s = matchedPoint.s +
                 projectionDx * tangentX +
                 projectionDy * tangentY;
    obstacle.l = lateralDx * normalX + lateralDy * normalY;
    obstacle.length = actor.length;
    obstacle.width = actor.width;
    return obstacle;
}

inline ObstacleCartesianPoint ToObstacleCartesianPoint(
    const rsim_plugin::ActorState& actor)
{
    ObstacleCartesianPoint point;
    point.x = actor.x;
    point.y = actor.y;
    point.heading = actor.h;
    point.speed = ActorPlanarSpeed(actor);
    point.accel = ActorLongitudinalAccel(actor);
    return point;
}

}  // namespace frenet_obstacle_perception_detail

class FrenetObstaclePerception
{
public:
    explicit FrenetObstaclePerception(
        const FrenetObstaclePerceptionConfig& config = {});

    const FrenetObstaclePerceptionConfig& config() const;
    void SetConfig(const FrenetObstaclePerceptionConfig& config);

    template <typename RefPointT>
    bool ConvertStaticObstacles(
        const std::vector<rsim_plugin::ActorState>& actors,
        int32_t egoActorId,
        const std::vector<RefPointT>& referencePoints,
        StaticFrenetObstaclePerceptionResult* result) const
    {
        if (result == nullptr || referencePoints.empty())
            return false;

        StaticFrenetObstaclePerceptionResult converted;
        converted.staticobstacles.reserve(actors.size());

        const double staticSpeedThreshold =
            std::max(0.0, perceptionConfig_.static_speed_threshold);
        for (const rsim_plugin::ActorState& actor : actors)
        {
            if (actor.id == egoActorId)
                continue;

            const double speed =
                frenet_obstacle_perception_detail::ActorPlanarSpeed(actor);
            if (speed <= staticSpeedThreshold)
            {
                converted.staticobstacles.push_back(
                    frenet_obstacle_perception_detail::ToStaticFrenetObstacle(
                        actor, referencePoints));
            }
        }

        *result = std::move(converted);
        return true;
    }

    template <typename RefPointT>
    bool ConvertDynamicObstacles(
        const std::vector<rsim_plugin::ActorState>& actors,
        int32_t egoActorId,
        const std::vector<RefPointT>& referencePoints,
        DynamicFrenetObstaclePerceptionResult* result) const
    {
        if (result == nullptr || referencePoints.empty())
            return false;

        DynamicFrenetObstaclePerceptionResult converted;
        converted.dynamicobstacles.reserve(actors.size());

        const double staticSpeedThreshold =
            std::max(0.0, perceptionConfig_.static_speed_threshold);
        for (const rsim_plugin::ActorState& actor : actors)
        {
            if (actor.id == egoActorId)
                continue;

            const double speed =
                frenet_obstacle_perception_detail::ActorPlanarSpeed(actor);
            if (speed <= staticSpeedThreshold)
                continue;

            const frenet_obstacle_perception_detail::ObstacleCartesianPoint point =
                frenet_obstacle_perception_detail::ToObstacleCartesianPoint(actor);
            CartesianFrenetState frenet;
            if (!cartesian_to_frenet_detail::CartesianPointToFrenet(
                    referencePoints, point, 0.0, &frenet))
            {
                return false;
            }

            DynamicFrenetObstacle obstacle;
            obstacle.id = actor.id;
            obstacle.dynamicfrenetstate = frenet;
            converted.dynamicobstacles.push_back(obstacle);
        }

        *result = std::move(converted);
        return true;
    }

private:
    FrenetObstaclePerceptionConfig perceptionConfig_;
};

}  // namespace rsim_driver
