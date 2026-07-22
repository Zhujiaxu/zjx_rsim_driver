#include "FrenetToCartesian.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
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

struct FrenetPoint
{
    double s = 0.0;
    double l = 0.0;
    double l_prime = 0.0;
    double l_double_prime = 0.0;
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
    const std::vector<RefPoint> straight = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
    };
    rsim_driver::CartesianPathPoint output;
    const FrenetPoint point{5.0, 2.0, 0.1, 0.02};
    if (!Require(rsim_driver::FrenetPointToCartesian(straight, point, &output) &&
                     Near(output.x, 5.0) && Near(output.y, 2.0) &&
                     Near(output.heading, std::atan2(0.1, 1.0)) &&
                     std::isfinite(output.kappa),
                 "regular Frenet state should convert with double heading delta"))
        return 1;

    const std::vector<RefPoint> singular = {
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 1.0, 0.0, 1.0},
    };
    if (!Require(!rsim_driver::FrenetPointToCartesian(
                     singular, FrenetPoint{0.5, 1.0, 0.0, 0.0}, &output),
                 "singular 1-kappa*l state should fail"))
        return 1;

    FrenetPoint invalid = point;
    invalid.l_prime = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!rsim_driver::FrenetPointToCartesian(straight, invalid, &output) &&
                     !rsim_driver::FrenetPointToCartesian(straight, point, nullptr),
                 "invalid Frenet state and null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS frenet_to_cartesian smoke\n");
    return 0;
}
