#include "CollisionCost.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

struct Obstacle
{
    double s;
    double l;
    double length;
    double width;
};

struct Boundary
{
    int id;
    double tin;
    double tout;
    double sinmin;
    double sinmax;
    double soutmin;
    double soutmax;
};

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

}  // namespace

int main()
{
    const rsim_driver::StaticCollisionCostConfig config;
    const std::vector<Obstacle> obstacles{{0.0, 0.0, 4.0, 2.0}};
    const double collision = rsim_driver::StaticObstacleCollisionCost(
        {0.0, 0.0}, obstacles, config);
    if (!Require(std::isinf(collision),
                 "overlapping static obstacle should have fatal cost"))
        return 1;

    const double touching = rsim_driver::StaticObstacleCollisionCost(
        {4.0, 0.0}, obstacles, config);
    if (!Require(std::isinf(touching),
                 "touching static envelopes should have fatal cost"))
        return 1;

    const double insideSafetyDistance =
        rsim_driver::StaticObstacleCollisionCost(
            {4.5, 0.0}, obstacles, config);
    if (!Require(std::isinf(insideSafetyDistance),
                 "surface clearance inside the collision distance should be fatal"))
        return 1;

    const double longitudinalRisk =
        rsim_driver::StaticObstacleCollisionCost(
            {5.55, 0.0}, obstacles, config);
    if (!Require(Near(longitudinalRisk, 0.5),
                 "longitudinal surface clearance should use linear risk cost"))
        return 1;

    const double lateralRisk =
        rsim_driver::StaticObstacleCollisionCost(
            {0.0, 3.55}, obstacles, config);
    if (!Require(Near(lateralRisk, 0.5),
                 "lateral surface clearance should use linear risk cost"))
        return 1;

    const double clear = rsim_driver::StaticObstacleCollisionCost(
        {6.5, 0.0}, obstacles, config);
    if (!Require(clear == 0.0,
                 "distant static obstacle should have zero cost"))
        return 1;

    rsim_driver::SlAxisAlignedBox first;
    rsim_driver::SlAxisAlignedBox second;
    if (!Require(rsim_driver::BuildSlAxisAlignedBox(
                     {0.0, 0.0}, 2.0, 2.0, &first) &&
                     rsim_driver::BuildSlAxisAlignedBox(
                         {5.0, 6.0}, 2.0, 2.0, &second),
                 "valid SL boxes should build") ||
        !Require(Near(rsim_driver::SlBoxClearanceDistance(first, second), 5.0),
                 "diagonal box clearance should use both axis gaps") ||
        !Require(!rsim_driver::SlBoxesOverlap(first, second),
                 "separated boxes should not overlap"))
    {
        return 1;
    }

    const std::vector<Obstacle> twoRiskObstacles{
        {0.0, 0.0, 4.0, 2.0},
        {11.1, 0.0, 4.0, 2.0},
    };
    const double accumulatedRisk =
        rsim_driver::StaticObstacleCollisionCost(
            {5.55, 0.0}, twoRiskObstacles, config);
    if (!Require(Near(accumulatedRisk, 1.0),
                 "soft costs from multiple obstacles should accumulate"))
        return 1;

    const std::vector<Obstacle> invalidObstacles{{0.0, 0.0, 0.0, 2.0}};
    if (!Require(std::isinf(rsim_driver::StaticObstacleCollisionCost(
                     {5.0, 0.0}, invalidObstacles, config)),
                 "invalid obstacle dimensions should fail closed"))
        return 1;

    rsim_driver::StaticCollisionCostConfig invalidConfig = config;
    invalidConfig.risk_distance = invalidConfig.collision_distance;
    const std::vector<Obstacle> noObstacles;
    if (!Require(std::isinf(rsim_driver::StaticObstacleCollisionCost(
                     {5.0, 0.0}, noObstacles, invalidConfig)),
                 "invalid collision configuration should fail closed"))
        return 1;

    const std::vector<Boundary> boundaries{{1, 0.0, 2.0, 4.0, 6.0, 4.0, 6.0}};
    const double dynamic_collision = rsim_driver::DynamicObstacleCollisionCost(
        {5.0, 1.0}, boundaries);
    if (!Require(std::isinf(dynamic_collision),
                 "point inside ST polygon should have fatal cost"))
        return 1;

    std::fprintf(stderr, "PASS collision_cost smoke\n");
    return 0;
}
