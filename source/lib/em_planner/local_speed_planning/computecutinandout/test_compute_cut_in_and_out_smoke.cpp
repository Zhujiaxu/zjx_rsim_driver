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
    if (!Require(computer.Compute(MakeReferenceLine(), obstacles, &result),
                 "cut in and out computation should succeed"))
        return 1;

    if (!Require(result.size() == 5,
                 "cut in and out output should keep one entry per dynamic obstacle"))
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

    result.push_back({});
    if (!Require(computer.Compute(MakeReferenceLine(),
                                  rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                                  &result) &&
                     result.empty(),
                 "empty dynamic obstacle result should succeed and clear output"))
        return 1;

    if (!Require(!computer.Compute(rsim_driver::localreferencelinepath{},
                                   obstacles,
                                   &result),
                 "empty local reference line should fail"))
        return 1;

    if (!Require(!computer.Compute(MakeReferenceLine(), obstacles, nullptr),
                 "null cut in and out output should fail"))
        return 1;

    std::fprintf(stderr, "PASS compute_cut_in_and_out smoke\n");
    return 0;
}
