#include "DynamicSpeedPlanner.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

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

rsim_driver::DynamicSpeedPlanStartPoint MakeStart(double speed = 1.0)
{
    rsim_driver::DynamicSpeedPlanStartPoint start;
    start.t = 0.0;
    start.s = 0.0;
    start.v = speed;
    start.a = 0.0;
    return start;
}

rsim_driver::DynamicSpeedPlanConfig MakeConfig()
{
    rsim_driver::DynamicSpeedPlanConfig config;
    config.time_step = 0.5;
    config.time_step_count = 4;
    config.s_step = 0.5;
    config.s_step_count = 8;
    config.reference_speed = 1.0;
    config.weight_reference_speed = 1.0;
    config.weight_acceleration = 0.0;
    config.weight_jerk = 0.0;
    config.weight_collision = 0.0;
    config.collision.collision_distance = 0.05;
    config.collision.risk_distance = 0.2;
    return config;
}

rsim_driver::localreferencelinepath MakeReferenceLine(double end_s,
                                                      double step = 0.5)
{
    rsim_driver::localreferencelinepath path;
    if (end_s < 0.0 || step <= 0.0)
        return path;

    for (double s = 0.0; s <= end_s + 1e-9; s += step)
        path.push_back({s, 0.0, 0.0, 0.0, s, 0.0});
    return path;
}

rsim_driver::DynamicFrenetObstacle MakeDynamicObstacle(double s,
                                                       double s_dot,
                                                       double l = 2.0,
                                                       double ldot = -1.0)
{
    rsim_driver::DynamicFrenetObstacle obstacle;
    obstacle.id = 101;
    obstacle.dynamicfrenetstate.s = s;
    obstacle.dynamicfrenetstate.s_dot = s_dot;
    obstacle.dynamicfrenetstate.l = l;
    obstacle.dynamicfrenetstate.ldot = ldot;
    return obstacle;
}

bool CheckBackwardEulerSpeed(
    const std::vector<rsim_driver::DynamicSpeedPoint>& points)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double dt = points[i].t - points[i - 1].t;
        if (dt <= 0.0)
            return false;
        const double expected_speed = (points[i].s - points[i - 1].s) / dt;
        if (!Near(points[i].v, expected_speed))
            return false;
    }
    return true;
}

bool CheckPreviousRowIndexes(
    const std::vector<rsim_driver::DynamicSpeedPoint>& points,
    double start_s,
    double s_step)
{
    if (points.empty() || points.front().rowindex != -1)
        return false;

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const int expected_previous_row =
            i == 1 ? 0
                   : static_cast<int>(std::llround(
                         (points[i - 1].s - start_s) / s_step)) -
                         1;
        if (points[i].rowindex != expected_previous_row)
            return false;
    }
    return true;
}

}  // namespace

int main()
{
    rsim_driver::DynamicSpeedPlanConfig config = MakeConfig();
    rsim_driver::DynamicSpeedPlanner planner(config);
    rsim_driver::DynamicSpeedPlanResult result;

    if (!Require(planner.Plan(MakeStart(),
                              MakeReferenceLine(10.0, config.s_step),
                              rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                              &result) &&
                     result.dpsuccess,
                 "planner should succeed without dynamic obstacles"))
        return 1;
    if (!Require(result.speed_points.size() == 5 &&
                     Near(result.speed_points.front().t, 0.0) &&
                     Near(result.speed_points.front().s, 0.0) &&
                     Near(result.speed_points.back().t, 2.0) &&
                     Near(result.speed_points.back().s, 2.0),
                 "unblocked planner should follow reference speed to time edge"))
        return 1;
    if (!Require(CheckBackwardEulerSpeed(result.speed_points),
                 "speed points should store backward Euler speeds"))
        return 1;
    for (const rsim_driver::DynamicSpeedPoint& point : result.speed_points)
    {
        if (!Require(Near(point.v, 1.0) &&
                         Near(point.a, 0.0) &&
                         Near(point.jerk, 0.0),
                     "reference-speed path should keep constant speed"))
            return 1;
    }
    if (!Require(CheckPreviousRowIndexes(result.speed_points,
                                         0.0,
                                         config.s_step),
                 "speed points should record selected previous row index"))
        return 1;

    rsim_driver::DynamicSpeedPlanConfig truncated_config = MakeConfig();
    truncated_config.time_step_count = 8;
    truncated_config.s_step_count = 10;
    planner.SetConfig(truncated_config);
    if (!Require(planner.Plan(MakeStart(),
                              MakeReferenceLine(1.0, truncated_config.s_step),
                              rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                              &result) &&
                     result.dpsuccess,
                 "planner should succeed with path length truncation"))
        return 1;
    if (!Require(result.speed_points.size() == 3 &&
                     Near(result.speed_points.back().t, 1.0) &&
                     Near(result.speed_points.back().s, 1.0),
                 "planner should stop on top edge without exceeding path length"))
        return 1;

    rsim_driver::DynamicSpeedPlanConfig local_choice_config = MakeConfig();
    local_choice_config.time_step = 1.0;
    local_choice_config.time_step_count = 2;
    local_choice_config.s_step = 1.0;
    local_choice_config.s_step_count = 4;
    local_choice_config.reference_speed = 2.0;
    local_choice_config.weight_reference_speed = 1.0;
    local_choice_config.weight_acceleration = 0.0;
    local_choice_config.weight_jerk = 0.0;
    planner.SetConfig(local_choice_config);
    if (!Require(planner.Plan(MakeStart(),
                              MakeReferenceLine(10.0, local_choice_config.s_step),
                              rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                              &result) &&
                     result.dpsuccess,
                 "planner should succeed with local predecessor selection"))
        return 1;
    if (!Require(result.speed_points.size() == 3 &&
                     Near(result.speed_points[1].s, 2.0) &&
                     Near(result.speed_points[2].s, 4.0) &&
                     result.speed_points[2].rowindex == 1,
                 "right-edge point should record best previous row by local cost"))
        return 1;

    rsim_driver::DynamicFrenetObstaclePerceptionResult dynamic_obstacles;
    dynamic_obstacles.dynamicobstacles = {
        MakeDynamicObstacle(2.0, 1.0, 2.0, -1.0),
    };
    local_choice_config.weight_collision = 1000.0;
    local_choice_config.collision.collision_distance = 0.1;
    local_choice_config.collision.risk_distance = 0.5;
    planner.SetConfig(local_choice_config);
    if (!Require(planner.Plan(MakeStart(),
                              MakeReferenceLine(10.0, local_choice_config.s_step),
                              dynamic_obstacles,
                              &result) &&
                     result.dpsuccess,
                 "planner should succeed when collision cost blocks a terminal point"))
        return 1;
    if (!Require(result.speed_points.size() == 3 &&
                     Near(result.speed_points.back().s, 3.0) &&
                     result.speed_points.back().rowindex == 0,
                 "collision cost should be part of local transition cost"))
        return 1;

    rsim_driver::DynamicSpeedPlanConfig invalid_config = MakeConfig();
    invalid_config.time_step = 0.0;
    planner.SetConfig(invalid_config);
    if (!Require(!planner.Plan(MakeStart(),
                               MakeReferenceLine(10.0, invalid_config.s_step),
                               rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                               &result) &&
                     !result.dpsuccess &&
                     result.speed_points.empty(),
                 "invalid config should fail and clear result"))
        return 1;

    planner.SetConfig(MakeConfig());
    if (!Require(!planner.Plan(MakeStart(),
                               MakeReferenceLine(0.0),
                               rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                               &result),
                 "zero path length should fail"))
        return 1;
    if (!Require(!planner.Plan(MakeStart(),
                               MakeReferenceLine(10.0),
                               rsim_driver::DynamicFrenetObstaclePerceptionResult{},
                               nullptr),
                 "null dynamic speed output should fail"))
        return 1;

    std::fprintf(stderr, "PASS dynamic_speed_planner smoke\n");
    return 0;
}
