#include "FrenetToCartesian.hpp"

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

struct SlPoint
{
    double s = 0.0;
    double l = 0.0;
};

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

std::vector<RefPoint> StraightReference()
{
    return {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
        {20.0, 0.0, 0.0, 0.0, 0.0, 20.0},
    };
}

}  // namespace

int main()
{
    const std::vector<RefPoint> reference = StraightReference();

    rsim_driver::CartesianPathPoint point;
    if (!Require(rsim_driver::FrenetPointToCartesian(reference, 5.0, 2.0, &point),
                 "single point conversion should succeed"))
        return 1;
    if (!Require(Near(point.x, 5.0, 1e-9) &&
                     Near(point.y, 2.0, 1e-9) &&
                     Near(point.heading, 0.0, 1e-9),
                 "straight reference should map l to positive y"))
        return 1;

    if (!Require(rsim_driver::FrenetPointToCartesian(reference, -5.0, 1.5, &point),
                 "front clamp conversion should succeed"))
        return 1;
    if (!Require(Near(point.x, 0.0, 1e-9) &&
                     Near(point.y, 1.5, 1e-9),
                 "s before reference should clamp to front point"))
        return 1;

    if (!Require(rsim_driver::FrenetPointToCartesian(reference, 25.0, -1.0, &point),
                 "back clamp conversion should succeed"))
        return 1;
    if (!Require(Near(point.x, 20.0, 1e-9) &&
                     Near(point.y, -1.0, 1e-9),
                 "s after reference should clamp to back point"))
        return 1;

    const std::vector<SlPoint> frenetPath = {
        {0.0, 0.0},
        {5.0, 0.0},
        {10.0, 1.0},
        {15.0, 1.0},
    };
    std::vector<rsim_driver::CartesianPathPoint> cartesianPath;
    if (!Require(rsim_driver::FrenetPathToCartesian(reference,
                                                    frenetPath,
                                                    &cartesianPath),
                 "path conversion should succeed"))
        return 1;
    if (!Require(cartesianPath.size() == frenetPath.size(),
                 "path conversion should preserve point count"))
        return 1;
    if (!Require(cartesianPath.front().heading >= -0.1 &&
                     cartesianPath.front().heading <= 0.1 &&
                     cartesianPath.back().heading >= -0.1 &&
                     cartesianPath.back().heading <= 0.1,
                 "path headings should follow forward motion"))
        return 1;

    std::fprintf(stderr, "PASS frenet_to_cartesian smoke\n");
    return 0;
}
