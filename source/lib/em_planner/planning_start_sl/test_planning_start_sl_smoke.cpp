#include "planning_start_sl/PlanningStartSl.hpp"

#include <cmath>
#include <cstdio>
#include <type_traits>
#include <utility>
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

template <typename T, typename = void>
struct HasCurvature : std::false_type
{
};

template <typename T>
struct HasCurvature<T, std::void_t<decltype(std::declval<T&>().curvature)>>
    : std::true_type
{
};

static_assert(!HasCurvature<rsim_driver::PlanningStartPoint>::value,
              "PlanningStartPoint should not require curvature");

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
    rsim_driver::PlanningStartPoint start;
    start.x = 3.0;
    start.y = 1.0;
    start.heading = 0.0;
    start.speed = 2.0;
    start.accel = 0.5;

    rsim_driver::PlanningStartFrenetState state;
    if (!Require(rsim_driver::ComputePlanningStartSl(start,
                                                     StraightReferenceLine(),
                                                     &state),
                 "straight planning start sl conversion should succeed"))
        return 1;
    if (!Require(Near(state.s, 3.0) &&
                     Near(state.l, 1.0) &&
                     Near(state.s_dot, 2.0) &&
                     Near(state.s_ddot, 0.5) &&
                     Near(state.l_prime, 0.0) &&
                     Near(state.l_double_prime, 0.0),
                 "straight planning start should convert to expected s/l"))
        return 1;

    const std::vector<RefPoint> curvedReference = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.2, 0.0, 10.0},
    };
    start.x = 5.0;
    start.y = 0.0;
    start.heading = std::atan(0.1);
    start.speed = 4.0;
    start.accel = 0.0;
    if (!Require(rsim_driver::ComputePlanningStartSl(start, curvedReference, &state),
                 "curved planning start sl conversion should succeed"))
        return 1;

    const double refK = 0.1;
    const double refDk = 0.0;
    const double refL = 0.0;
    const double alpha = 1.0 - refK * refL;
    const double tanDelta = 0.1;
    const double deltaTheta = std::atan(tanDelta);
    const double cosDelta = std::cos(deltaTheta);
    const double sinDelta = std::sin(deltaTheta);
    const double expectedSDot = start.speed * cosDelta / alpha;
    const double expectedLDot = start.speed * sinDelta;
    const double expectedSDdot =
        (start.accel * cosDelta +
         refDk * refL * expectedSDot * expectedSDot +
         2.0 * refK * expectedSDot * expectedLDot) /
        alpha;
    const double expectedLDdot =
        start.accel * sinDelta -
        refK * alpha * expectedSDot * expectedSDot;
    const double expectedLDoublePrime =
        (expectedLDdot - tanDelta * expectedSDdot) /
        (expectedSDot * expectedSDot);
    if (!Require(Near(state.s, 5.0) &&
                     Near(state.l, 0.0) &&
                     Near(state.l_prime, tanDelta) &&
                     Near(state.l_double_prime, expectedLDoublePrime, 1e-9) &&
                     Near(state.s_ddot, expectedSDdot, 1e-9),
                 "projected reference curvature should affect l'' and s_ddot"))
        return 1;

    if (!Require(!rsim_driver::ComputePlanningStartSl(
                     start, std::vector<RefPoint>{}, &state),
                 "empty reference points should fail"))
        return 1;

    std::fprintf(stderr, "PASS planning_start_sl smoke\n");
    return 0;
}
