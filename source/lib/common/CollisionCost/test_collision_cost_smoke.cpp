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

}  // namespace

int main()
{
    const std::vector<Obstacle> obstacles{{5.0, 0.0, 4.0, 2.0}};
    const double collision = rsim_driver::StaticObstacleCollisionCost(
        {5.0, 0.0}, obstacles);
    if (!Require(std::isinf(collision),
                 "overlapping static obstacle should have fatal cost"))
        return 1;

    const double clear = rsim_driver::StaticObstacleCollisionCost(
        {20.0, 0.0}, obstacles);
    if (!Require(clear == 0.0,
                 "distant static obstacle should have zero cost"))
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
