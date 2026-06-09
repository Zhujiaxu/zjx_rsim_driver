#include "planning_start/PlanningStartPoint.hpp"

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

static_assert(HasCurvature<rsim_driver::PlanningStartPoint>::value,
              "PlanningStartPoint should carry planning-start curvature");

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
    rsim_driver::PlanningStartResult result;
    result.start_point.x = 3.0;
    result.start_point.y = 1.0;
    result.start_point.heading = 0.0;
    result.start_point.curvature = 0.012;
    result.start_point.speed = 2.0;
    result.start_point.accel = 0.5;
    result.start_point.source = rsim_driver::PlanningStartSource::KinematicExtrapolation;
    result.start_curvature = 0.012;

    rsim_driver::PlanningStartFrenetState state;
    if (!Require(result.ToFrenet(StraightReferenceLine(), &state),
                 "straight planning start frenet conversion should succeed"))
        return 1;
    if (!Require(Near(state.s, 3.0) &&
                     Near(state.l, 1.0) &&
                     Near(state.s_dot, 2.0) &&
                     Near(state.s_ddot, 0.5) &&
                     Near(state.l_prime, 0.0) &&
                     Near(state.l_double_prime, 0.0) &&
                     Near(state.curvature, 0.0),
                 "kinematic planning start should convert to expected s/l and zero curvature"))
        return 1;

    const std::vector<RefPoint> curvedReference = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.2, 0.0, 10.0},
    };
    result.start_point.x = 5.0;
    result.start_point.y = 0.0;
    result.start_point.heading = std::atan(0.1);
    result.start_point.curvature = 999.0;
    result.start_point.speed = 4.0;
    result.start_point.accel = 0.0;
    result.start_point.source = rsim_driver::PlanningStartSource::PreviousTrajectory;
    result.start_curvature = 0.02;
    if (!Require(result.ToFrenet(curvedReference, &state),
                 "curved planning start frenet conversion should succeed"))
        return 1;

    const double refK = 0.1;
    const double refL = 0.0;
    const double alpha = 1.0 - refK * refL;
    const double tanDelta = 0.1;
    const double deltaTheta = std::atan(tanDelta);
    const double expectedSDot = result.start_point.speed * std::cos(deltaTheta) / alpha;
    if (!Require(Near(state.s, 5.0) &&
                     Near(state.l, 0.0) &&
                     Near(state.s_dot, expectedSDot) &&
                     Near(state.l_prime, tanDelta) &&
                     Near(state.curvature, result.start_curvature),
                 "previous-trajectory planning start should convert stable fields and use result curvature"))
        return 1;

    if (!Require(!result.ToFrenet(std::vector<RefPoint>{}, &state),
                 "empty reference points should fail"))
        return 1;

    std::fprintf(stderr, "PASS planning_start_frenet smoke\n");
    return 0;
}
