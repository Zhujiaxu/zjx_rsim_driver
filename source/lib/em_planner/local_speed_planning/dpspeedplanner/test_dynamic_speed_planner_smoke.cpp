#include "DynamicSpeedPlanner.hpp"

#include <cmath>
#include <cstdio>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-6)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DynamicPlanSpeedConfig MakeConfig()
{
    rsim_driver::DynamicPlanSpeedConfig config;
    config.time_step = 0.5;
    config.time_step_count = 4;
    config.s_step = 0.5;
    config.s_step_count = 20;
    config.reference_speed = 1.0;
    config.weight_reference_speed = 1.0;
    config.weight_acceleration = 1.0;
    config.weight_jerk = 1.0;
    config.weight_collision = 1.0;
    return config;
}

rsim_driver::DynamicPlanSpeedPoint MakeStart()
{
    return {0.0, 0.0, 1.0, 0.0};
}

rsim_driver::localreferencelinepath MakeReferenceLine(double length,
                                                       double step = 0.5)
{
    rsim_driver::localreferencelinepath line;
    for (double s = 0.0; s < length - 1e-9; s += step)
        line.push_back({s, 0.0, 0.0, 0.0, s, 0.0});
    line.push_back({length, 0.0, 0.0, 0.0, length, 0.0});
    return line;
}

}  // namespace

int main()
{
    rsim_driver::DynamicPlanSpeedPlanner planner(MakeConfig());
    rsim_driver::DynamicPlanSpeedResult result;
    if (!Require(planner.Plan(MakeStart(), MakeReferenceLine(10.0), {}, &result) &&
                     result.dpsuccess &&
                     result.termination ==
                         rsim_driver::SpeedPlanTermination::TimeHorizon,
                 "unblocked coarse planner should finish on time horizon"))
        return 1;
    if (!Require(result.stpoints.size() == 5 &&
                     Near(result.terminal_time, 2.0) &&
                     Near(result.terminal_s, 2.0),
                 "time-horizon result should preserve the expected coarse path"))
        return 1;

    if (!Require(planner.Plan(MakeStart(), MakeReferenceLine(1.1), {}, &result) &&
                     result.termination ==
                         rsim_driver::SpeedPlanTermination::SpatialHorizon &&
                     Near(result.terminal_s, 1.1),
                 "spatial grid should include an exact non-step-aligned end"))
        return 1;

    rsim_driver::DynamicPlanSpeedConfig local = MakeConfig();
    local.time_step = 1.0;
    local.time_step_count = 2;
    local.s_step = 1.0;
    local.s_step_count = 4;
    local.reference_speed = 2.0;
    local.weight_acceleration = 0.0;
    local.weight_jerk = 0.0;
    planner.SetConfig(local);
    if (!Require(planner.Plan({0.0, 0.0, 2.0, 0.0},
                              MakeReferenceLine(4.0, 1.0), {}, &result) &&
                     result.stpoints.size() == 3 &&
                     Near(result.stpoints[1].s, 2.0) &&
                     Near(result.stpoints[2].s, 4.0),
                 "local predecessor selection behavior should remain stable"))
        return 1;

    local = MakeConfig();
    local.collisionconfig.collision_distance = 0.0;
    local.collisionconfig.risk_distance = 0.0;
    planner.SetConfig(local);
    rsim_driver::CutInAndOutInfo far_boundary;
    far_boundary.tin = 0.0;
    far_boundary.tout = 2.0;
    far_boundary.sinmin = 5.0;
    far_boundary.sinmax = 10.0;
    far_boundary.soutmin = 5.0;
    far_boundary.soutmax = 10.0;
    rsim_driver::CutInAndOutInfo blocking_boundary;
    blocking_boundary.tin = 1.0;
    blocking_boundary.tout = 2.0;
    blocking_boundary.sinmin = 0.0;
    blocking_boundary.sinmax = 5.0;
    blocking_boundary.soutmin = 0.0;
    blocking_boundary.soutmax = 5.0;
    if (!Require(planner.Plan(MakeStart(), MakeReferenceLine(100.0),
                              {far_boundary, blocking_boundary}, &result) &&
                     result.termination ==
                         rsim_driver::SpeedPlanTermination::BlockedHorizon &&
                     Near(result.terminal_time, 0.5),
                 "blocked search should return its latest reachable horizon"))
        return 1;

    rsim_driver::DynamicPlanSpeedConfig invalid = MakeConfig();
    invalid.s_step = 0.0;
    planner.SetConfig(invalid);
    if (!Require(!planner.Plan(MakeStart(), MakeReferenceLine(10.0), {}, &result) &&
                     !result.dpsuccess && result.stpoints.empty(),
                 "invalid configuration should fail and clear output"))
        return 1;
    invalid = MakeConfig();
    invalid.weight_acceleration = -1.0;
    planner.SetConfig(invalid);
    if (!Require(!planner.Plan(MakeStart(), MakeReferenceLine(10.0), {},
                               &result),
                 "negative cost weights should fail instead of being clamped"))
        return 1;
    if (!Require(!planner.Plan(MakeStart(), MakeReferenceLine(10.0), {}, nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS dynamic_speed_planner smoke\n");
    return 0;
}
