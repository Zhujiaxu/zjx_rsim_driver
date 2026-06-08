#include "CartesianToFrenet.hpp"

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

struct PlanningStart
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double curvature = 0.0;
    double speed = 0.0;
    double accel = 0.0;
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

std::vector<RefPoint> StraightReferenceLine()
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
    rsim_driver::CartesianFrenetState state;

    PlanningStart straightStart;
    straightStart.x = 12.0;
    straightStart.y = 3.0;
    straightStart.heading = 0.0;
    straightStart.speed = 5.0;
    straightStart.accel = 1.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), straightStart, &state),
                 "straight conversion should succeed"))
        return 1;
    if (!Require(Near(state.s, 12.0) &&
                 Near(state.l, 3.0) &&
                 Near(state.s_dot, 5.0) &&
                 Near(state.s_ddot, 1.0) &&
                 Near(state.l_prime, 0.0) &&
                 Near(state.l_double_prime, 0.0),
                 "straight conversion should compute refined s and positive l"))
        return 1;

    PlanningStart rightSideStart = straightStart;
    rightSideStart.y = -2.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), rightSideStart, &state),
                 "right-side conversion should succeed"))
        return 1;
    if (!Require(Near(state.l, -2.0),
                 "l should be negative on the matched point right side"))
        return 1;

    PlanningStart headingOffsetStart;
    headingOffsetStart.x = 10.0;
    headingOffsetStart.y = 0.0;
    headingOffsetStart.heading = std::atan(0.1);
    headingOffsetStart.speed = 2.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), headingOffsetStart, &state),
                 "heading-offset conversion should succeed"))
        return 1;
    if (!Require(Near(state.l_prime, 0.1),
                 "heading offset should produce l prime"))
        return 1;

    std::vector<RefPoint> curvedReference = {
        {0.0, 0.0, 0.0, 0.1, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.1, 0.0, 10.0},
        {20.0, 0.0, 0.0, 0.1, 0.0, 20.0},
    };
    PlanningStart curvedStart;
    curvedStart.x = 0.0;
    curvedStart.y = 0.0;
    curvedStart.heading = 0.0;
    curvedStart.curvature = 0.2;
    curvedStart.speed = 3.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     curvedReference, curvedStart, &state),
                 "curved conversion should succeed"))
        return 1;
    if (!Require(Near(state.l_double_prime, 0.1),
                 "curvature difference should produce l double prime"))
        return 1;

    const std::vector<RefPoint> emptyReference;
    if (!Require(!rsim_driver::CartesianToFrenet(
                     emptyReference, straightStart, &state),
                 "empty reference line should fail"))
        return 1;

    PlanningStart singularStart;
    singularStart.x = 0.0;
    singularStart.y = 0.0;
    singularStart.heading = 0.5 * 3.14159265358979323846;
    if (!Require(!rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), singularStart, &state),
                 "near-perpendicular heading should fail"))
        return 1;

    std::fprintf(stderr, "PASS cartesian_to_frenet smoke\n");
    return 0;
}
