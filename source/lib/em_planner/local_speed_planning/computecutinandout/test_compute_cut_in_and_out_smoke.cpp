#include "computecutinandout.hpp"

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

rsim_driver::localreferencelinepath MakeReferenceLine()
{
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 10.0, 0.0},
    };
}

rsim_driver::DynamicFrenetObstacle MakeObstacle(int32_t id,
                                                double s,
                                                double sDot,
                                                double l,
                                                double ldot)
{
    rsim_driver::DynamicFrenetObstacle obstacle;
    obstacle.id = id;
    obstacle.dynamicfrenetstate.s = s;
    obstacle.dynamicfrenetstate.s_dot = sDot;
    obstacle.dynamicfrenetstate.l = l;
    obstacle.dynamicfrenetstate.ldot = ldot;
    return obstacle;
}

}  // namespace

int main()
{
    rsim_driver::DynamicFrenetObstaclePerceptionResult obstacles;
    obstacles.dynamicobstacles = {
        MakeObstacle(101, 10.0, 5.0, 2.0, -1.0),
        MakeObstacle(102, 20.0, 2.0, -2.0, 1.0),
        MakeObstacle(103, 30.0, 4.0, 0.0, 1.0),
        MakeObstacle(104, 40.0, 3.0, 2.0, 1.0),
        MakeObstacle(105, 50.0, 3.0, 2.0, 0.0),
    };

    rsim_driver::ComputeCutInAndOut computer;
    std::vector<rsim_driver::CutInAndOutInfo> result;
    std::vector<rsim_driver::VirtualObstacleSeed> seeds;
    if (!Require(computer.Compute(MakeReferenceLine(),
                                  obstacles,
                                  8.0,
                                  0.05,
                                  8.0,
                                  &result,
                                  &seeds),
                 "cut in and out computation should succeed"))
        return 1;

    if (!Require(result.size() == 5,
                 "cut in and out output should keep one entry per dynamic obstacle"))
        return 1;
    if (!Require(seeds.empty(),
                 "ordinary dynamic obstacles should not produce virtual obstacle seeds"))
        return 1;

    if (!Require(result[0].id == 101 &&
                     Near(result[0].tin, 1.0) &&
                     Near(result[0].tout, 3.0) &&
                     Near(result[0].sin, 15.0) &&
                     Near(result[0].sout, 25.0),
                 "left-side obstacle should use min and max boundary times"))
        return 1;

    if (!Require(result[1].id == 102 &&
                     Near(result[1].tin, 1.0) &&
                     Near(result[1].tout, 3.0) &&
                     Near(result[1].sin, 22.0) &&
                     Near(result[1].sout, 26.0),
                 "right-side obstacle should use min and max boundary times"))
        return 1;

    if (!Require(result[2].id == 103 &&
                     Near(result[2].tin, 0.0) &&
                     Near(result[2].tout, 1.0) &&
                     Near(result[2].sin, 30.0) &&
                     Near(result[2].sout, 34.0),
                 "opposite-sign boundary times should set tin to zero"))
        return 1;

    if (!Require(result[3].id == 104 &&
                     Near(result[3].tin, -3.0) &&
                     Near(result[3].tout, -1.0) &&
                     std::isinf(result[3].sin) &&
                     std::isinf(result[3].sout),
                 "both-negative boundary times should set sin and sout to infinity"))
        return 1;

    if (!Require(result[4].id == 105 &&
                     std::isinf(result[4].tin) &&
                     std::isinf(result[4].tout) &&
                     std::isinf(result[4].sin) &&
                     std::isinf(result[4].sout),
                 "zero ldot should produce infinity times and s values"))
        return 1;

    obstacles.dynamicobstacles = {
        MakeObstacle(106, 12.0, 1.0, 0.0, 0.0),
    };
    if (!Require(computer.Compute(MakeReferenceLine(),
                                  obstacles,
                                  2.0,
                                  0.05,
                                  8.0,
                                  &result,
                                  &seeds) &&
                     result.size() == 1 &&
                     seeds.empty() &&
                     Near(result[0].tin, 0.0) &&
                     Near(result[0].sin, 12.0) &&
                     std::isfinite(result[0].tout) &&
                     std::isfinite(result[0].sout),
                 "zero ldot lane-overlap obstacle should produce finite ST occupancy"))
        return 1;

    obstacles.dynamicobstacles = {
        MakeObstacle(201, 12.0, 1.0, 0.0, 0.0),
        MakeObstacle(202, 16.0, -2.0, 0.0, 0.0),
        MakeObstacle(203, 8.0, 3.0, 0.0, 0.0),
    };
    if (!Require(computer.Compute(MakeReferenceLine(),
                                  obstacles,
                                  8.0,
                                  0.05,
                                  8.0,
                                  &result,
                                  &seeds),
                 "virtual obstacle seed computation should succeed"))
        return 1;
    if (!Require(result.size() == 1 && result.front().id == 203,
                 "seeded obstacle ids should be removed from cut in and out output"))
        return 1;
    if (!Require(seeds.size() == 2 &&
                     seeds[0].source_actor_id == 201 &&
                     seeds[0].type == rsim_driver::VirtualObstacleType::SlowLead &&
                     Near(seeds[0].longitudinal_buffer, 0.05) &&
                     Near(seeds[0].lateral_buffer, 0.5) &&
                     seeds[1].source_actor_id == 202 &&
                     seeds[1].type == rsim_driver::VirtualObstacleType::OncomingConflict &&
                     Near(seeds[1].longitudinal_buffer, -0.1) &&
                     Near(seeds[1].lateral_buffer, 0.5),
                 "slow lead and oncoming obstacles should produce expected seeds"))
        return 1;

    result.push_back({});
    seeds.push_back({});
    if (!Require(computer.Compute(MakeReferenceLine(),
                                  rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                                  8.0,
                                  0.05,
                                  8.0,
                                  &result,
                                  &seeds) &&
                     result.empty(),
                 "empty dynamic obstacle result should succeed and clear cut output"))
        return 1;
    if (!Require(seeds.empty(),
                 "empty dynamic obstacle result should clear seed output"))
        return 1;

    if (!Require(!computer.Compute(rsim_driver::localreferencelinepath{},
                                   obstacles,
                                   8.0,
                                   0.05,
                                   8.0,
                                   &result,
                                   &seeds),
                 "empty local reference line should fail"))
        return 1;

    if (!Require(!computer.Compute(MakeReferenceLine(),
                                   obstacles,
                                   8.0,
                                   0.05,
                                   8.0,
                                   nullptr,
                                   &seeds),
                 "null cut in and out output should fail"))
        return 1;
    if (!Require(!computer.Compute(MakeReferenceLine(),
                                   obstacles,
                                   8.0,
                                   0.05,
                                   8.0,
                                   &result,
                                   nullptr),
                 "null virtual obstacle seed output should fail"))
        return 1;

    std::fprintf(stderr, "PASS compute_cut_in_and_out smoke\n");
    return 0;
}
