#include "perception/StaticObstacleDetector.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

namespace
{

bool IsControlledActor(int32_t actorId,
                       const std::vector<int32_t>& controlledActorIds)
{
    return std::find(controlledActorIds.begin(),
                     controlledActorIds.end(),
                     actorId) != controlledActorIds.end();
}

double ActorPlanarSpeed(const rsim_plugin::ActorState& actor)
{
    const double velocitySpeed =
        std::sqrt(actor.vel_x * actor.vel_x + actor.vel_y * actor.vel_y);
    return std::max(std::fabs(actor.speed), velocitySpeed);
}

StaticObstacle ToStaticObstacle(const rsim_plugin::ActorState& actor)
{
    StaticObstacle obstacle;
    obstacle.id = actor.id;
    obstacle.type = actor.type;
    obstacle.x = actor.x;
    obstacle.y = actor.y;
    obstacle.heading = actor.h;
    obstacle.speed = ActorPlanarSpeed(actor);
    obstacle.length = actor.length;
    obstacle.width = actor.width;
    obstacle.height = actor.height;
    return obstacle;
}

}  // namespace

std::vector<StaticObstacle> DetectStaticObstacles(
    const std::vector<rsim_plugin::ActorState>& actors,
    const std::vector<int32_t>& controlledActorIds,
    const StaticObstacleConfig& config)
{
    const double maxStaticSpeed = std::max(0.0, config.max_static_speed);
    std::vector<StaticObstacle> obstacles;
    obstacles.reserve(actors.size());

    for (const rsim_plugin::ActorState& actor : actors)
    {
        if (IsControlledActor(actor.id, controlledActorIds))
            continue;
        if (ActorPlanarSpeed(actor) > maxStaticSpeed)
            continue;

        obstacles.push_back(ToStaticObstacle(actor));
    }

    return obstacles;
}

}  // namespace rsim_driver
