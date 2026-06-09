#include "perception/StaticObstacleDetector.hpp"

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

rsim_plugin::ActorState MakeActor(int32_t id,
                                  double x,
                                  double speed,
                                  double velX = 0.0)
{
    rsim_plugin::ActorState actor{};
    actor.id = id;
    actor.type = 0;
    actor.x = x;
    actor.y = 0.0;
    actor.h = 0.0;
    actor.speed = speed;
    actor.vel_x = velX;
    actor.length = 4.5;
    actor.width = 2.0;
    actor.height = 1.5;
    return actor;
}

}  // namespace

int main()
{
    const std::vector<rsim_plugin::ActorState> actors = {
        MakeActor(1, 0.0, 0.0),
        MakeActor(2, 5.0, 0.05),
        MakeActor(3, 10.0, 0.2),
        MakeActor(4, 15.0, 0.0, 0.2),
    };
    const std::vector<int32_t> controlledIds = {1};

    rsim_driver::StaticObstacleConfig config;
    config.max_static_speed = 0.1;
    std::vector<rsim_driver::StaticObstacle> obstacles =
        rsim_driver::DetectStaticObstacles(actors, controlledIds, config);

    if (!Require(obstacles.size() == 1 && obstacles.front().id == 2,
                 "detector should exclude controlled and moving actors"))
        return 1;
    if (!Require(obstacles.front().length > 4.0 && obstacles.front().width > 1.0,
                 "detector should preserve obstacle dimensions"))
        return 1;

    config.max_static_speed = 0.25;
    obstacles = rsim_driver::DetectStaticObstacles(actors, controlledIds, config);
    if (!Require(obstacles.size() == 3,
                 "static speed threshold should be configurable"))
        return 1;

    std::fprintf(stderr, "PASS static_obstacle_detector smoke\n");
    return 0;
}
