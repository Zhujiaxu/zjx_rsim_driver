#include "CartesianToFrenet.hpp"

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

struct CartesianPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
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
    const std::vector<RefPoint> reference = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
    };
    rsim_driver::CartesianFrenetState state;
    if (!Require(rsim_driver::cartesian_to_frenet_detail::CartesianPointToFrenet(
                     reference, CartesianPoint{5.0, 2.0, 0.0, 3.0, 0.0},
                     0.0, &state) &&
                     Near(state.s, 5.0) && Near(state.l, 2.0) &&
                     Near(state.s_dot, 3.0) && Near(state.l_dot, 0.0),
                 "straight reference conversion should preserve finite dynamics"))
        return 1;

    if (!Require(!rsim_driver::cartesian_to_frenet_detail::CartesianPointToFrenet(
                     reference,
                     CartesianPoint{5.0, 0.0, 1.5707963267948966, 3.0, 0.0},
                     0.0, &state),
                 "perpendicular heading should fail at the Frenet singularity"))
        return 1;

    CartesianPoint invalid{5.0, 0.0, 0.0, 3.0, 0.0};
    invalid.speed = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!rsim_driver::cartesian_to_frenet_detail::CartesianPointToFrenet(
                     reference, invalid, 0.0, &state) &&
                     !rsim_driver::cartesian_to_frenet_detail::CartesianPointToFrenet(
                         reference, CartesianPoint{}, 0.0, nullptr),
                 "non-finite dynamics and null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS cartesian_to_frenet smoke\n");
    return 0;
}
