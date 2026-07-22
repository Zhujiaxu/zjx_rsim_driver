#include "FrenetObstaclePerception.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
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
    rsim_driver::StaticFrenetObstaclePerceptionResult staticResult;
    if (!Require(perception.ConvertStaticObstacles(actors,
                                                   1,
                                                   StraightReferenceLine(),
                                                   &staticResult),
                 "static frenet obstacle perception should succeed"))
        return 1;

    if (!Require(staticResult.staticobstacles.size() == 1 &&
                     staticResult.staticobstacles.front().id == 101 &&
                     Near(staticResult.staticobstacles.front().s, 12.0) &&
                     Near(staticResult.staticobstacles.front().l, 1.2) &&
                     Near(staticResult.staticobstacles.front().length, 4.5) &&
                     Near(staticResult.staticobstacles.front().width, 2.0),
                 "static obstacle should be ego-filtered and converted to s/l with size"))
        return 1;

    rsim_driver::DynamicFrenetObstaclePerceptionResult dynamicResult;
    if (!Require(perception.ConvertDynamicObstacles(actors,
                                                    1,
                                                    StraightReferenceLine(),
                                                    &dynamicResult),
                 "dynamic frenet obstacle perception should succeed"))
        return 1;

    if (!Require(dynamicResult.dynamicobstacles.size() == 1 &&
                     dynamicResult.dynamicobstacles.front().id == 102 &&
                     Near(dynamicResult.dynamicobstacles.front().dynamicfrenetstate.s, 8.0) &&
                     Near(dynamicResult.dynamicobstacles.front().dynamicfrenetstate.l, -0.8) &&
                     Near(dynamicResult.dynamicobstacles.front().dynamicfrenetstate.s_dot, 4.0),
                 "dynamic obstacle should output full frenet state"))
        return 1;

    const std::vector<rsim_plugin::ActorState> seedActors = {
        MakeActor(1, 0.0, 0.0),
        MakeActor(201, 10.0, 0.2, 0.0, 1.0),
        MakeActor(202, 16.0, -0.2, 3.14159265358979323846, 4.0),
    };
    if (!Require(perception.ConvertDynamicObstacles(seedActors,
                                                    1,
                                                    StraightReferenceLine(),
                                                    &dynamicResult),
                 "dynamic conversion should not require virtual obstacle seed output"))
        return 1;

    std::vector<rsim_driver::VirtualObstacleSeed> seeds = {
        {201, rsim_driver::VirtualObstacleType::SlowLead, 0.5, 0.5},
        {202, rsim_driver::VirtualObstacleType::OncomingConflict, 8.0, 0.5},
    };
    std::vector<rsim_driver::VirtualFrenetObstacle> virtualResult;
    if (!Require(perception.ConvertVirtualObstacles(seedActors,
                                                    1,
                                                    StraightReferenceLine(),
                                                    seeds,
                                                    &virtualResult),
                 "virtual obstacle seeds should resolve on current reference line"))
        return 1;
    if (!Require(virtualResult.size() == 2 &&
                     virtualResult[0].id == 201 &&
                     virtualResult[1].id == 202 &&
                     Near(virtualResult[0].length, 5.0) &&
                     Near(virtualResult[0].width, 2.5) &&
                     Near(virtualResult[1].length, 12.5) &&
                     Near(virtualResult[1].width, 2.5),
                 "resolved virtual obstacles should keep source ids and enlarged sizes"))
        return 1;

    std::vector<rsim_plugin::ActorState> invalidActors = seedActors;
    invalidActors[1].length = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!perception.ConvertDynamicObstacles(invalidActors,
                                                     1,
                                                     StraightReferenceLine(),
                                                     &dynamicResult),
                 "invalid actor geometry should fail"))
        return 1;

    if (!Require(!perception.ConvertStaticObstacles(actors,
                                                    1,
                                                    std::vector<RefPoint>{},
                                                    &staticResult),
                 "empty static reference points should fail"))
        return 1;

    if (!Require(!perception.ConvertDynamicObstacles(actors,
                                                     1,
                                                     std::vector<RefPoint>{},
                                                     &dynamicResult),
                 "empty dynamic reference points should fail"))
        return 1;

    std::fprintf(stderr, "PASS frenet_obstacle_perception smoke\n");
    return 0;
}
