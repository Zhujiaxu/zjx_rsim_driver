#include "computecutinandout.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
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
    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {20.0, 0.0, 0.0, 0.0, 20.0, 0.0}};
}

rsim_driver::DynamicFrenetObstacle MakeObstacle(double s,
                                                 double s_dot,
                                                 double l,
                                                 double l_dot)
{
    rsim_driver::DynamicFrenetObstacle obstacle;
    obstacle.id = 10;
    obstacle.dynamicfrenetstate.s = s;
    obstacle.dynamicfrenetstate.s_dot = s_dot;
    obstacle.dynamicfrenetstate.l = l;
    obstacle.dynamicfrenetstate.l_dot = l_dot;
    obstacle.length = 4.0;
    obstacle.width = 2.0;
    return obstacle;
}

}  // namespace

int main()
{
    rsim_driver::ComputeCutInAndOut computer;
    std::vector<rsim_driver::CutInAndOutInfo> result;
    std::vector<rsim_driver::VirtualObstacleSeed> seeds;

    rsim_driver::DynamicFrenetObstaclePerceptionResult obstacles;
    obstacles.dynamicobstacles = {MakeObstacle(10.0, 5.0, 2.0, -1.0)};
    if (!Require(computer.Compute(MakeReferenceLine(), obstacles, 8.0, 0.05,
                                  8.0, &result, &seeds) &&
                     result.size() == 1 && Near(result[0].tin, 1.0) &&
                     Near(result[0].tout, 3.0) && Near(result[0].sin, 15.0) &&
                     Near(result[0].sinmin, 13.0) &&
                     Near(result[0].sinmax, 17.0),
                 "moving obstacle should use longitudinal length in ST bounds"))
        return 1;

    obstacles.dynamicobstacles = {MakeObstacle(12.0, 1.0, 0.0, 0.0)};
    if (!Require(computer.Compute(MakeReferenceLine(), obstacles, 8.0, 0.05,
                                  8.0, &result, &seeds) &&
                     result.size() == 1 && Near(result[0].tin, 0.0) &&
                     Near(result[0].sinmin, 10.0) &&
                     Near(result[0].sinmax, 14.0),
                 "stationary lateral-overlap obstacle should occupy the horizon"))
        return 1;

    obstacles.dynamicobstacles = {MakeObstacle(12.0, 1.0, 3.0, 0.0)};
    if (!Require(computer.Compute(MakeReferenceLine(), obstacles, 8.0, 0.05,
                                  8.0, &result, &seeds) && result.empty(),
                 "non-overlapping obstacle should be omitted instead of using sentinels"))
        return 1;

    obstacles.dynamicobstacles.front().length =
        std::numeric_limits<double>::quiet_NaN();
    if (!Require(!computer.Compute(MakeReferenceLine(), obstacles, 8.0, 0.05,
                                   8.0, &result, &seeds) && result.empty(),
                 "invalid obstacle geometry should fail"))
        return 1;

    if (!Require(!computer.Compute({}, obstacles, 8.0, 0.05, 8.0,
                                   &result, &seeds) &&
                     !computer.Compute(MakeReferenceLine(), obstacles, 8.0,
                                       0.05, 8.0, nullptr, &seeds) &&
                     !computer.Compute(MakeReferenceLine(), obstacles, 8.0,
                                       0.05, 8.0, &result, nullptr),
                 "invalid outputs and reference line should fail"))
        return 1;

    std::fprintf(stderr, "PASS compute_cut_in_and_out smoke\n");
    return 0;
}
