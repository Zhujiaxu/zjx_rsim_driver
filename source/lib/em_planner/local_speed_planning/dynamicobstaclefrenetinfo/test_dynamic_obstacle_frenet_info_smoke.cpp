#include "dynamicobsfrenet.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_plugin::ActorState MakeActor(int32_t id,
                                  double x,
                                  double y,
                                  double heading = 0.0,
                                  double speed = 0.0,
                                  double velX = 0.0,
                                  double velY = 0.0)
{
    rsim_plugin::ActorState actor{};
    actor.id = id;
    actor.x = x;
    actor.y = y;
    actor.h = heading;
    actor.speed = speed;
    actor.vel_x = velX;
    actor.vel_y = velY;
    actor.length = 4.5;
    actor.width = 2.0;
    actor.height = 1.5;
    return actor;
}

rsim_driver::localreferencelinepath StraightLocalReferenceLine()
{
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 10.0, 0.0},
        {20.0, 0.0, 0.0, 0.0, 20.0, 0.0},
    };
}

}  // namespace

int main()
{
    const std::vector<rsim_plugin::ActorState> actors = {
        MakeActor(1, 0.0, 0.0),
        MakeActor(101, 12.0, 1.2, 0.0, 0.0),
        MakeActor(102, 8.0, -0.8, 0.0, 4.0, 4.0),
    };

    rsim_driver::DynamicObstacleFrenetInfo dynamicObstacleFrenetInfo;
    rsim_driver::DynamicFrenetObstaclePerceptionResult result;
    if (!Require(dynamicObstacleFrenetInfo.ConvertDynamicObstacles(
                     actors,
                     1,
                     StraightLocalReferenceLine(),
                     &result),
                 "dynamic obstacle frenet info conversion should succeed"))
        return 1;

    if (!Require(result.obstacles.size() == 1 &&
                     result.obstacles.front().id == 102 &&
                     Near(result.obstacles.front().dynamicfrenetstate.s, 8.0) &&
                     Near(result.obstacles.front().dynamicfrenetstate.l, -0.8) &&
                     Near(result.obstacles.front().dynamicfrenetstate.s_dot, 4.0),
                 "dynamic obstacle should be converted with local reference line"))
        return 1;

    rsim_driver::FrenetObstaclePerceptionConfig config;
    config.static_speed_threshold = 5.0;
    dynamicObstacleFrenetInfo.SetConfig(config);
    if (!Require(Near(dynamicObstacleFrenetInfo.config().static_speed_threshold,
                      5.0),
                 "dynamic obstacle frenet info should expose perception config"))
        return 1;

    if (!Require(dynamicObstacleFrenetInfo.ConvertDynamicObstacles(
                     actors,
                     1,
                     StraightLocalReferenceLine(),
                     &result) &&
                     result.obstacles.empty(),
                 "configured static speed threshold should filter dynamic obstacles"))
        return 1;

    if (!Require(!dynamicObstacleFrenetInfo.ConvertDynamicObstacles(
                     actors,
                     1,
                     rsim_driver::localreferencelinepath{},
                     &result),
                 "empty local reference line should fail"))
        return 1;

    if (!Require(!dynamicObstacleFrenetInfo.ConvertDynamicObstacles(
                     actors,
                     1,
                     StraightLocalReferenceLine(),
                     nullptr),
                 "null dynamic obstacle output should fail"))
        return 1;

    std::fprintf(stderr, "PASS dynamic_obstacle_frenet_info smoke\n");
    return 0;
}
