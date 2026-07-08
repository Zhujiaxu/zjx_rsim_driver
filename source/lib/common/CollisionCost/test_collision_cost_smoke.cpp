#include "CollisionCost.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
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

struct CutInAndOutLine
{
    int id = 0;
    double tin = 0.0;
    double tout = 0.0;
    double sinmin = 0.0;
    double sinmax = 0.0;
    double soutmin = 0.0;
    double soutmax = 0.0;
};

struct StaticObstacleBox
{
    double s = 0.0;
    double l = 0.0;
    double length = 0.0;
    double width = 0.0;
};

}  // namespace

int main()
{
    rsim_driver::StaticCollisionCostConfig config;
    config.collision_distance = 0.5;
    config.risk_distance = 1.5;
    config.infinity_cost = 1000.0;

    const std::vector<rsim_driver::SlPoint> obstacles = {
        {0.0, 0.0},
        {0.0, 2.0},
    };

    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({0.0, 4.0}, obstacles, config), 0.0),
                 "point outside risk range should have zero collision cost"))
        return 1;

    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({0.0, 1.0}, obstacles, config), 1.0),
                 "two finite obstacle costs should sum"))
        return 1;

    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({0.0, 0.25}, obstacles, config),
                      config.infinity_cost),
                 "point inside collision distance should return infinity cost"))
        return 1;

    config.infinity_cost = std::numeric_limits<double>::infinity();
    if (!Require(std::isinf(rsim_driver::StaticObstacleCollisionCost({0.0, 0.0}, obstacles, config)),
                 "infinite infinity cost should be supported"))
        return 1;

    config.collision_distance = 0.5;
    config.risk_distance = 1.5;
    config.infinity_cost = 1000.0;
    const std::vector<StaticObstacleBox> boxes = {
        {0.0, 0.0, 4.0, 2.0},
    };
    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({1.0, 0.0}, boxes, config),
                      config.infinity_cost),
                 "point inside static obstacle box should return infinity cost"))
        return 1;
    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({3.0, 0.0}, boxes, config),
                      0.5),
                 "static obstacle box should measure risk distance from box edge"))
        return 1;
    if (!Require(Near(rsim_driver::StaticObstacleCollisionCost({4.0, 0.0}, boxes, config),
                      0.0),
                 "point outside static obstacle box risk range should have zero cost"))
        return 1;

    rsim_driver::DynamicCollisionCostConfig dynamicConfig;
    dynamicConfig.collision_distance = 0.5;
    dynamicConfig.risk_distance = 1.5;
    dynamicConfig.infinity_cost = 1000.0;
    const std::vector<CutInAndOutLine> cutLines = {
        {1, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0},
    };
    if (!Require(Near(rsim_driver::DynamicObstacleCollisionCost({4.0, 1.0},
                                                                cutLines,
                                                                dynamicConfig),
                      0.0),
                 "st point outside dynamic risk range should have zero cost"))
        return 1;
    if (!Require(Near(rsim_driver::DynamicObstacleCollisionCost({2.75, 1.0},
                                                                cutLines,
                                                                dynamicConfig),
                      0.75),
                 "dynamic obstacle should use point to st polygon distance"))
        return 1;
    if (!Require(Near(rsim_driver::DynamicObstacleCollisionCost({1.0, 1.0},
                                                                cutLines,
                                                                dynamicConfig),
                      dynamicConfig.infinity_cost),
                 "st point inside dynamic polygon should return infinity cost"))
        return 1;

    std::fprintf(stderr, "PASS collision_cost smoke\n");
    return 0;
}
