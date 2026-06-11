#include "CollisionCost.hpp"

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

}  // namespace

int main()
{
    rsim_driver::CollisionCostConfig config;
    config.collision_distance = 0.5;
    config.risk_distance = 1.5;
    config.infinity_cost = 1000.0;

    const std::vector<rsim_driver::SlPoint> obstacles = {
        {0.0, 0.0},
        {0.0, 2.0},
    };

    if (!Require(Near(rsim_driver::ObstacleCollisionCost({0.0, 4.0}, obstacles, config), 0.0),
                 "point outside risk range should have zero collision cost"))
        return 1;

    if (!Require(Near(rsim_driver::ObstacleCollisionCost({0.0, 1.0}, obstacles, config), 1.0),
                 "two finite obstacle costs should sum"))
        return 1;

    if (!Require(Near(rsim_driver::ObstacleCollisionCost({0.0, 0.25}, obstacles, config),
                      config.infinity_cost),
                 "point inside collision distance should return infinity cost"))
        return 1;

    config.infinity_cost = std::numeric_limits<double>::infinity();
    if (!Require(std::isinf(rsim_driver::ObstacleCollisionCost({0.0, 0.0}, obstacles, config)),
                 "infinite infinity cost should be supported"))
        return 1;

    std::fprintf(stderr, "PASS collision_cost smoke\n");
    return 0;
}
