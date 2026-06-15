#include "drivable_area/DrivableArea.hpp"

#include <cmath>
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

std::vector<rsim_driver::DpPathPoint> MakePath(double l = 0.0)
{
    return {
        {0.0, l, 0.0, 0.0},
        {1.0, l, 0.0, 0.0},
        {2.0, l, 0.0, 0.0},
    };
}

rsim_driver::StaticFrenetObstacle MakeObstacle(double s,
                                               double l,
                                               double length,
                                               double width)
{
    rsim_driver::StaticFrenetObstacle obstacle;
    obstacle.id = 1;
    obstacle.s = s;
    obstacle.l = l;
    obstacle.length = length;
    obstacle.width = width;
    return obstacle;
}

}  // namespace

int main()
{
    rsim_driver::DrivableAreaConfig config;
    config.left_road_boundary_l = 3.5;
    config.right_road_boundary_l = -3.5;
    config.obstacle_lateral_buffer = 0.0;
    rsim_driver::DrivableAreaBuilder builder(config);
    rsim_driver::DrivableArea area;

    if (!Require(builder.Build(MakePath(), {}, &area),
                 "empty obstacles should build drivable area"))
        return 1;
    if (!Require(area.left_boundary.size() == 3 &&
                     area.right_boundary.size() == 3,
                 "drivable area should match coarse path size"))
        return 1;
    for (std::size_t i = 0; i < area.left_boundary.size(); ++i)
    {
        if (!Require(Near(area.left_boundary[i].l, 3.5) &&
                         Near(area.right_boundary[i].l, -3.5) &&
                         Near(area.left_boundary[i].s, static_cast<double>(i)) &&
                         Near(area.right_boundary[i].s, static_cast<double>(i)),
                     "empty obstacles should keep configured road boundaries"))
            return 1;
    }

    const std::vector<rsim_driver::StaticFrenetObstacle> centerObstacle = {
        MakeObstacle(1.0, 0.0, 2.0, 1.0),
    };
    if (!Require(builder.Build(MakePath(1.2), centerObstacle, &area),
                 "coarse path left of obstacle should build"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, 1.0),
                     "coarse path left of obstacle should raise right boundary"))
            return 1;
    }
    for (const rsim_driver::SlPoint& point : area.left_boundary)
    {
        if (!Require(Near(point.l, 3.5),
                     "left road boundary should stay configured"))
            return 1;
    }

    if (!Require(builder.Build(MakePath(-1.2), centerObstacle, &area),
                 "coarse path right of obstacle should build"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.left_boundary)
    {
        if (!Require(Near(point.l, -1.0),
                     "coarse path right of obstacle should lower left boundary"))
            return 1;
    }
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, -3.5),
                     "right road boundary should stay configured"))
            return 1;
    }

    const std::vector<rsim_driver::StaticFrenetObstacle> multipleObstacles = {
        MakeObstacle(1.0, -0.5, 2.0, 1.0),
        MakeObstacle(1.0, 0.2, 2.0, 1.0),
    };
    if (!Require(builder.Build(MakePath(1.2), multipleObstacles, &area),
                 "multiple obstacles should build"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, 1.2),
                     "multiple obstacles should keep most restrictive right boundary"))
            return 1;
    }

    const std::vector<rsim_driver::StaticFrenetObstacle> blockingObstacle = {
        MakeObstacle(1.0, 0.0, 8.0, 1.0),
    };
    if (!Require(!builder.Build(MakePath(0.1), blockingObstacle, &area) &&
                     area.left_boundary.empty() &&
                     area.right_boundary.empty(),
                 "crossed boundaries should fail and clear output"))
        return 1;

    std::fprintf(stderr, "PASS drivable_area smoke\n");
    return 0;
}
