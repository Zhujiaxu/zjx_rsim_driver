#include "PlanningStartPoint.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{

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

rsim_plugin::ActorState MakeEgo()
{
    rsim_plugin::ActorState ego{};
    ego.id = 1;
    ego.x = 0.0;
    ego.y = 0.0;
    ego.h = 0.0;
    ego.speed = 10.0;
    ego.acc_x = -2.0;
    ego.acc_y = 0.0;
    ego.length = 4.5;
    ego.width = 2.0;
    return ego;
}

}  // namespace

int main()
{
    rsim_driver::PlanningStartConfig config;
    config.planningPeriod = 0.1;
    config.mismatchDistanceThreshold = 0.3;
    rsim_driver::PlanningStart planning_start(config);
    rsim_driver::PlanningStartResult result;

    const rsim_plugin::ActorState ego = MakeEgo();
    if (!Require(planning_start.Compute(ego, 1.0, {}, &result) &&
                     result.start_point.source ==
                         rsim_driver::PlanningStartSource::KinematicExtrapolation &&
                     Near(result.start_point.x, 0.99) &&
                     Near(result.start_point.speed, 9.8) &&
                     Near(result.start_point.accel, -2.0),
                 "kinematic start should preserve signed braking acceleration"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> previous = {
        {0.0, 0.0, 0.0, 0.01, 10.0, -1.0, 1.0},
        {1.0, 0.0, 0.0, 0.02, 9.9, -1.0, 1.1},
        {1.98, 0.0, 0.0, 0.03, 9.8, -1.0, 1.2},
    };
    if (!Require(planning_start.Compute(ego, 1.0, previous, &result) &&
                     result.start_point.source ==
                         rsim_driver::PlanningStartSource::PreviousTrajectory &&
                     Near(result.start_point.time, 1.1) &&
                     Near(result.start_curvature, 0.02),
                 "matching previous trajectory should be reused"))
        return 1;

    previous[1].time = previous[0].time;
    if (!Require(!planning_start.Compute(ego, 1.0, previous, &result),
                 "non-increasing history time should fail"))
        return 1;
    rsim_plugin::ActorState invalid = ego;
    invalid.acc_x = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!planning_start.Compute(invalid, 1.0, {}, &result) &&
                     !planning_start.Compute(ego, 1.0, {}, nullptr),
                 "invalid ego and null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS planning_start smoke\n");
    return 0;
}
