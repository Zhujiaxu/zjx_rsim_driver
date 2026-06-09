#include "obstacle_sl/ObstacleSlConverter.hpp"

#include <cmath>
#include <cstdio>
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

}  // namespace

int main()
{
    const std::vector<RefPoint> referencePoints = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
        {20.0, 0.0, 0.0, 0.0, 0.0, 20.0},
    };
    const std::vector<rsim_driver::StaticObstacle> staticObstacles = {
        {101, 0, 12.0, 1.2, 0.0, 0.0, 4.5, 2.0, 1.5},
        {102, 0, 8.0, -0.8, 0.0, 0.0, 4.5, 2.0, 1.5},
    };

    std::vector<rsim_driver::SlObstacle> slObstacles;
    if (!Require(rsim_driver::ConvertStaticObstaclesToSl(
                     staticObstacles, referencePoints, &slObstacles),
                 "obstacle sl conversion should succeed"))
        return 1;

    if (!Require(slObstacles.size() == 2 &&
                 slObstacles[0].id == 101 &&
                 Near(slObstacles[0].s, 12.0) &&
                 Near(slObstacles[0].l, 1.2) &&
                 slObstacles[1].id == 102 &&
                 Near(slObstacles[1].s, 8.0) &&
                 Near(slObstacles[1].l, -0.8),
                 "straight reference conversion should produce expected s/l"))
        return 1;

    if (!Require(!rsim_driver::ConvertStaticObstaclesToSl(
                     staticObstacles, std::vector<RefPoint>{}, &slObstacles),
                 "empty reference points should fail"))
        return 1;

    std::fprintf(stderr, "PASS obstacle_sl_converter smoke\n");
    return 0;
}
