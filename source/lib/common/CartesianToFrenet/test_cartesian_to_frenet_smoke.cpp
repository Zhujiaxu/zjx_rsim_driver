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

enum class TestPlanningStartSource
{
    KinematicExtrapolation,
    PreviousTrajectory
};

struct PlanningStart
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    TestPlanningStartSource source = TestPlanningStartSource::KinematicExtrapolation;
};

struct PlanningStartResult
{
    PlanningStart start_point;
    double start_curvature = 0.0;
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

PlanningStartResult MakeResult(PlanningStart start,
                               TestPlanningStartSource source =
                                   TestPlanningStartSource::KinematicExtrapolation,
                               double startCurvature = 0.0)
{
    start.source = source;

    PlanningStartResult result;
    result.start_point = start;
    result.start_curvature = startCurvature;
    return result;
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
                     StraightReferenceLine(),
                     MakeResult(straightStart,
                                TestPlanningStartSource::KinematicExtrapolation,
                                0.25),
                     &state),
                 "straight conversion should succeed"))
        return 1;
    if (!Require(Near(state.s, 12.0) &&
                     Near(state.l, 3.0) &&
                     Near(state.s_dot, 5.0) &&
                     Near(state.s_ddot, 1.0) &&
                     Near(state.l_prime, 0.0) &&
                     Near(state.l_double_prime, 0.0) &&
                     Near(state.curvature, 0.0),
                 "kinematic conversion should compute refined s, positive l and zero curvature"))
        return 1;

    PlanningStart rightSideStart = straightStart;
    rightSideStart.y = -2.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), MakeResult(rightSideStart), &state),
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
                     StraightReferenceLine(), MakeResult(headingOffsetStart), &state),
                 "heading-offset conversion should succeed"))
        return 1;
    if (!Require(Near(state.l_prime, 0.1),
                 "heading offset should produce l prime"))
        return 1;

    std::vector<RefPoint> curvedReference = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.2, 0.0, 10.0},
    };
    PlanningStart curvedStart;
    curvedStart.x = 5.0;
    curvedStart.y = 0.0;
    curvedStart.heading = std::atan(0.1);
    curvedStart.speed = 4.0;
    const double previousStartCurvature = 0.02;
    if (!Require(rsim_driver::CartesianToFrenet(
                     curvedReference,
                     MakeResult(curvedStart,
                                TestPlanningStartSource::PreviousTrajectory,
                                previousStartCurvature),
                     &state),
                 "curved reference conversion should succeed"))
        return 1;

    const double refK = 0.1;
    const double refL = 0.0;
    const double alpha = 1.0 - refK * refL;
    const double tanDelta = 0.1;
    const double deltaTheta = std::atan(tanDelta);
    const double cosDelta = std::cos(deltaTheta);
    const double expectedSDot = curvedStart.speed * cosDelta / alpha;
    if (!Require(Near(state.s, 5.0) &&
                     Near(state.s_dot, expectedSDot) &&
                     Near(state.l_prime, tanDelta) &&
                     Near(state.curvature, previousStartCurvature),
                 "previous-trajectory conversion should compute stable fields and use start curvature"))
        return 1;

    PlanningStart stoppedStart;
    stoppedStart.x = 5.0;
    stoppedStart.y = 0.0;
    stoppedStart.heading = std::atan(0.2);
    stoppedStart.speed = 0.0;
    stoppedStart.accel = 1.0;
    if (!Require(rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), MakeResult(stoppedStart), &state),
                 "stopped conversion should succeed"))
        return 1;
    if (!Require(Near(state.s_dot, 0.0) &&
                     Near(state.s_ddot, stoppedStart.accel *
                                             std::cos(stoppedStart.heading)) &&
                     Near(state.l_prime, 0.2) &&
                     Near(state.l_double_prime, 0.0),
                 "stopped conversion should zero l double prime"))
        return 1;

    const std::vector<RefPoint> emptyReference;
    if (!Require(!rsim_driver::CartesianToFrenet(
                     emptyReference, MakeResult(straightStart), &state),
                 "empty reference line should fail"))
        return 1;

    PlanningStart singularStart;
    singularStart.x = 0.0;
    singularStart.y = 0.0;
    singularStart.heading = 0.5 * 3.14159265358979323846;
    if (!Require(!rsim_driver::CartesianToFrenet(
                     StraightReferenceLine(), MakeResult(singularStart), &state),
                 "near-perpendicular heading should fail"))
        return 1;

    std::fprintf(stderr, "PASS cartesian_to_frenet smoke\n");
    return 0;
}
