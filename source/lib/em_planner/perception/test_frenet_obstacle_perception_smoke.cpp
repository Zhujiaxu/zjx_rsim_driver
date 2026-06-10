#include "perception/FrenetObstaclePerception.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

struct RefPoint
{
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
    double k = 0.0;
    double dk = 0.0;
    double s = 0.0;
};

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

std::vector<RefPoint> StraightReferenceLine()
{
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
        {20.0, 0.0, 0.0, 0.0, 0.0, 20.0},
    };
}

}  // namespace

int main()
{
    const std::vector<rsim_plugin::ActorState> actors = {
        MakeActor(1, 0.0, 0.0),
        MakeActor(101, 12.0, 1.2, 1.57, 0.0),
        MakeActor(102, 8.0, -0.8, 0.0, 4.0, 4.0),
    };

    rsim_driver::FrenetObstaclePerception perception;
    rsim_driver::FrenetObstaclePerceptionResult result;
    if (!Require(perception.Convert(actors, 1, StraightReferenceLine(), &result),
                 "frenet obstacle perception should succeed"))
        return 1;

    if (!Require(result.static_obstacles.size() == 1 &&
                     result.static_obstacles.front().id == 101 &&
                     Near(result.static_obstacles.front().s, 12.0) &&
                     Near(result.static_obstacles.front().l, 1.2),
                 "static obstacle should be ego-filtered and converted to s/l"))
        return 1;

    if (!Require(result.dynamic_obstacles.size() == 1 &&
                     result.dynamic_obstacles.front().id == 102 &&
                     Near(result.dynamic_obstacles.front().frenet.s, 8.0) &&
                     Near(result.dynamic_obstacles.front().frenet.l, -0.8) &&
                     Near(result.dynamic_obstacles.front().frenet.s_dot, 4.0),
                 "dynamic obstacle should output full frenet state"))
        return 1;

    if (!Require(!perception.Convert(actors, 1, std::vector<RefPoint>{}, &result),
                 "empty reference points should fail"))
        return 1;

    std::fprintf(stderr, "PASS frenet_obstacle_perception smoke\n");
    return 0;
}
