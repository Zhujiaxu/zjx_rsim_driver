#include "DrivableArea.hpp"

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

rsim_driver::StaticObsFrenetState MakeObstacle(double s,
                                               double l,
                                               double length,
                                               double width)
{
    rsim_driver::StaticObsFrenetState obstacle;
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
    config.approach_longitudinal_buffer = 0.0;
    config.departure_longitudinal_buffer = 0.0;
    config.obstacle_transition_length = 0.0;
    config.collision_clearance = 0.0;
    rsim_driver::DrivableAreaBuilder builder(config);
    rsim_driver::DrivableAreaResult area;

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

    const std::vector<rsim_driver::StaticObsFrenetState> centerObstacle = {
        MakeObstacle(1.0, 0.0, 2.0, 1.0),
    };
    if (!Require(builder.Build(MakePath(1.2), centerObstacle, &area),
                 "coarse path left of obstacle should build"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, 0.5),
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
        if (!Require(Near(point.l, -0.5),
                     "coarse path right of obstacle should lower left boundary"))
            return 1;
    }
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, -3.5),
                     "right road boundary should stay configured"))
            return 1;
    }

    const std::vector<rsim_driver::DpPathPoint> bracketedObstaclePath = {
        {0.0, 0.0, 0.0, 0.0},
        {1.0, 2.2, 0.0, 0.0},
        {2.0, 2.2, 0.0, 0.0},
    };
    const std::vector<rsim_driver::StaticObsFrenetState> bracketedObstacle = {
        MakeObstacle(0.5, 1.0, 1.2, 1.0),
    };
    if (!Require(builder.Build(bracketedObstaclePath, bracketedObstacle, &area),
                 "bracketed obstacle should build"))
        return 1;
    if (!Require(Near(area.right_boundary[0].l, 1.5) &&
                     Near(area.right_boundary[1].l, 1.5) &&
                     Near(area.right_boundary[2].l, -3.5),
                 "bracketed obstacle should use average coarse l for right boundary"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.left_boundary)
    {
        if (!Require(Near(point.l, 3.5),
                     "bracketed obstacle should keep left boundary configured"))
            return 1;
    }

    const std::vector<rsim_driver::StaticObsFrenetState> multipleObstacles = {
        MakeObstacle(1.0, -0.5, 2.0, 1.0),
        MakeObstacle(1.0, 0.2, 2.0, 1.0),
    };
    if (!Require(builder.Build(MakePath(1.2), multipleObstacles, &area),
                 "multiple obstacles should build"))
        return 1;
    for (const rsim_driver::SlPoint& point : area.right_boundary)
    {
        if (!Require(Near(point.l, 0.7),
                     "multiple obstacles should keep most restrictive right boundary"))
            return 1;
    }

    rsim_driver::DrivableAreaConfig clearanceConfig = config;
    clearanceConfig.approach_longitudinal_buffer = 0.0;
    clearanceConfig.departure_longitudinal_buffer = 0.0;
    clearanceConfig.obstacle_transition_length = 0.0;
    clearanceConfig.collision_clearance = 0.6;
    rsim_driver::DrivableAreaBuilder clearanceBuilder(clearanceConfig);
    const std::vector<rsim_driver::StaticObsFrenetState> clearanceObstacle = {
        MakeObstacle(1.0, 0.0, 0.2, 1.0),
    };
    if (!Require(clearanceBuilder.Build(MakePath(1.5), clearanceObstacle, &area),
                 "hard collision clearance should build") ||
        !Require(Near(area.right_boundary[0].l, -3.5) &&
                     Near(area.right_boundary[1].l, 1.1) &&
                     Near(area.right_boundary[2].l, -3.5),
                 "rounded hard clearance should expand the obstacle at overlap"))
    {
        return 1;
    }

    const std::vector<rsim_driver::DpPathPoint> roundedClearancePath = {
        {0.6, 1.5, 0.0, 0.0},
    };
    const double expectedLateralClearance =
        std::sqrt(0.6 * 0.6 - 0.3 * 0.3);
    if (!Require(clearanceBuilder.Build(
                     roundedClearancePath, clearanceObstacle, &area),
                 "rounded clearance at a longitudinal gap should build") ||
        !Require(Near(area.right_boundary.front().l,
                      0.5 + expectedLateralClearance),
                 "0.3 m longitudinal gap should use Euclidean lateral clearance"))
    {
        return 1;
    }

    const std::vector<rsim_driver::StaticObsFrenetState> invalidObstacle = {
        MakeObstacle(1.0, 0.0, 0.0, 1.0),
    };
    if (!Require(!builder.Build(MakePath(), invalidObstacle, &area) &&
                     area.left_boundary.empty() &&
                     area.right_boundary.empty(),
                 "invalid obstacle geometry should fail closed"))
        return 1;

    rsim_driver::DrivableAreaConfig transitionConfig = config;
    transitionConfig.obstacle_transition_length = 1.0;
    rsim_driver::DrivableAreaBuilder transitionBuilder(transitionConfig);
    const std::vector<rsim_driver::DpPathPoint> transitionPath = {
        {0.0, 1.5, 0.0, 0.0},
        {1.0, 1.5, 0.0, 0.0},
        {2.0, 1.5, 0.0, 0.0},
        {3.0, 1.5, 0.0, 0.0},
        {4.0, 1.5, 0.0, 0.0},
    };
    const std::vector<rsim_driver::StaticObsFrenetState> transitionObstacle = {
        MakeObstacle(2.0, 0.0, 0.2, 1.0),
    };
    if (!Require(transitionBuilder.Build(
                     transitionPath, transitionObstacle, &area),
                 "transition corridor should build") ||
        !Require(Near(area.right_boundary[0].l, -3.5) &&
                     Near(area.right_boundary[1].l, -3.1) &&
                     Near(area.right_boundary[2].l, 0.5) &&
                     Near(area.right_boundary[3].l, -3.1) &&
                     Near(area.right_boundary[4].l, -3.5),
                 "transition corridor should tighten and release continuously"))
    {
        return 1;
    }

    const std::vector<rsim_driver::StaticObsFrenetState> blockingObstacle = {
        MakeObstacle(1.0, 0.0, 8.0, 8.0),
    };
    if (!Require(!builder.Build(MakePath(0.1), blockingObstacle, &area) &&
                     area.left_boundary.empty() &&
                     area.right_boundary.empty(),
                 "crossed boundaries should fail and clear output"))
        return 1;

    std::fprintf(stderr, "PASS drivable_area smoke\n");
    return 0;
}
