#include "SpeedQpOptimizer.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 2e-4)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DynamicPlanSpeedResult MakeCoarse(
    double terminal_time,
    rsim_driver::SpeedPlanTermination termination)
{
    rsim_driver::DynamicPlanSpeedResult result;
    result.dpsuccess = true;
    result.termination = termination;
    result.terminal_time = terminal_time;
    result.terminal_s = terminal_time;
    result.stpoints = {
        {0.0, 0.0, 1.0, 0.0},
        {terminal_time, terminal_time, 1.0, 0.0},
    };
    return result;
}

rsim_driver::StDrivableArea MakeArea(const std::vector<double> &times,
                                     double length)
{
    rsim_driver::StDrivableArea area;
    for (double time : times)
    {
        area.lower_boundary.push_back({0.0, time});
        area.upper_boundary.push_back({length, time});
    }
    return area;
}

bool CheckKinematics(const std::vector<rsim_driver::DynamicPlanSpeedPoint> &points)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const auto &previous = points[i - 1];
        const auto &current = points[i];
        const double dt = current.t - previous.t;
        const double position_residual =
            current.s - previous.s - dt * previous.v -
            dt * dt * previous.a / 3.0 - dt * dt * current.a / 6.0;
        const double velocity_residual =
            current.v - previous.v - dt * previous.a / 2.0 -
            dt * current.a / 2.0;
        if (!Near(position_residual, 0.0) ||
            !Near(velocity_residual, 0.0))
            return false;
    }
    return true;
}

bool CheckDenseSamples(
    const std::vector<rsim_driver::DynamicPlanSpeedPoint> &points,
    double time_step)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const auto &start = points[i - 1];
        const auto &end = points[i];
        const double dt = end.t - start.t;
        const double jerk = (end.a - start.a) / dt;
        double previous_s = start.s;
        for (double tau = std::min(time_step, dt);
             tau <= dt + 1e-9;
             tau = std::min(tau + time_step, dt))
        {
            const double v = start.v + start.a * tau +
                             0.5 * jerk * tau * tau;
            const double s = start.s + start.v * tau +
                             0.5 * start.a * tau * tau +
                             jerk * tau * tau * tau / 6.0;
            if (v < -rsim_driver::speed_qp_numerics::
                        kSolutionNoiseTolerance ||
                s + rsim_driver::speed_qp_numerics::
                        kSolutionNoiseTolerance < previous_s)
            {
                return false;
            }
            previous_s = s;
            if (std::fabs(tau - dt) <= 1e-9)
                break;
        }
    }
    return true;
}

}  // namespace

int main()
{
    rsim_driver::SpeedQpOptimizerConfig config;
    config.nominal_time_step = 1.0;
    config.reference_speed = 1.0;
    config.longitudinal_safety_buffer = 0.0;
    config.weight_progress = 10.0;
    config.weight_coarse_s = 10.0;
    rsim_driver::SpeedQpOptimizer optimizer(config);

    auto coarse = MakeCoarse(2.2, rsim_driver::SpeedPlanTermination::TimeHorizon);
    std::vector<double> grid;
    if (!Require(rsim_driver::BuildSpeedQpTimeGrid(coarse, 1.0, &grid) &&
                     grid.size() == 4 && Near(grid[0], 0.0) &&
                     Near(grid[1], 1.0) && Near(grid[2], 2.0) &&
                     Near(grid[3], 2.2),
                 "time grid should preserve an exact partial terminal interval"))
        return 1;

    rsim_driver::SpeedQpResult result;
    if (!Require(optimizer.Optimize(coarse, MakeArea(grid, 10.0), 10.0, &result) &&
                     result.qpsuccess && result.stpoints.size() == grid.size(),
                 "variable-step speed QP should solve"))
        return 1;
    if (!Require(CheckKinematics(result.stpoints),
                 "QP output should satisfy variable-step kinematics"))
        return 1;
    if (!Require(CheckDenseSamples(result.stpoints, config.dense_time_step),
                 "constant-jerk dense samples should move forward"))
        return 1;
    if (!Require(result.stpoints.front().s == coarse.stpoints.front().s &&
                     result.stpoints.front().v == coarse.stpoints.front().v,
                 "QP output should restore exact initial position and speed"))
        return 1;
    for (std::size_t i = 0; i < result.stpoints.size(); ++i)
    {
        if (!Require(Near(result.stpoints[i].t, grid[i]) &&
                         result.stpoints[i].v >= 0.0 &&
                         (i == 0 || result.stpoints[i].s >=
                                            result.stpoints[i - 1].s),
                     "QP output should use the requested grid and move forward"))
            return 1;
    }

    coarse = MakeCoarse(0.6, rsim_driver::SpeedPlanTermination::BlockedHorizon);
    grid.clear();
    if (!Require(rsim_driver::BuildSpeedQpTimeGrid(coarse, 1.0, &grid) &&
                     grid.size() == 2 && Near(grid.back(), 0.6) &&
                     optimizer.Optimize(coarse, MakeArea(grid, 10.0), 10.0,
                                        &result) &&
                     !result.terminal_constraint_applied,
                 "blocked horizon should use its exact partial QP interval"))
        return 1;

    coarse = MakeCoarse(2.2, rsim_driver::SpeedPlanTermination::TimeHorizon);
    grid = {0.0, 1.0, 2.0, 2.2};
    coarse.stpoints.front().a = -6.0;
    if (!Require(optimizer.Optimize(coarse, MakeArea(grid, 10.0), 10.0,
                                    &result) &&
                     result.stpoints.front().a >= config.a_min &&
                     result.stpoints.front().a <= config.a_max,
                 "initial acceleration should be optimized as bounded control"))
        return 1;
    for (std::size_t i = 0; i < result.stpoints.size(); ++i)
    {
        if (!Require(result.stpoints[i].a >= config.a_min - 1e-8 &&
                         result.stpoints[i].a <= config.a_max + 1e-8,
                     "future acceleration nodes should respect control bounds"))
            return 1;
    }

    coarse = MakeCoarse(2.2, rsim_driver::SpeedPlanTermination::SpatialHorizon);
    if (!Require(optimizer.Optimize(coarse, MakeArea(grid, 2.2), 2.2, &result) &&
                     result.terminal_constraint_applied &&
                     result.stpoints.back().s == 2.2,
                 "spatial-horizon QP should enforce its terminal distance"))
        return 1;

    coarse.dpsuccess = true;
    coarse.termination = rsim_driver::SpeedPlanTermination::SpatialHorizon;
    coarse.terminal_time = 1.6;
    coarse.terminal_s = 28.0;
    coarse.stpoints = {
        {0.0, 0.0, 4.65, -0.22},
        {1.0, 9.0, 15.0, 20.0},
        {1.6, 28.0, 41.0, 45.0},
    };
    grid.clear();
    if (!Require(rsim_driver::BuildSpeedQpTimeGrid(coarse, 1.0, &grid) &&
                     optimizer.Optimize(coarse, MakeArea(grid, 28.0), 28.0,
                                        &result) &&
                     !result.terminal_constraint_applied &&
                     result.stpoints.back().s < 28.0,
                 "unreachable spatial terminal should remain a soft target"))
        return 1;

    coarse = MakeCoarse(2.2, rsim_driver::SpeedPlanTermination::TimeHorizon);
    grid = {0.0, 1.0, 2.0, 2.2};
    coarse.terminal_s = 0.0;
    coarse.stpoints.back().s = 0.0;
    coarse.stpoints.front().v = 0.0;
    coarse.stpoints.back().v = 0.0;
    if (!Require(optimizer.Optimize(coarse, MakeArea(grid, 2.0), 10.0,
                                    &result),
                 "active zero-speed constraints should solve"))
        return 1;
    for (const auto &point : result.stpoints)
    {
        if (!Require(point.s == 0.0 && point.v == 0.0,
                     "solver-scale negative zero should be canonicalized"))
            return 1;
    }

    rsim_driver::StDrivableArea mismatched = MakeArea(grid, 10.0);
    mismatched.upper_boundary.pop_back();
    if (!Require(!optimizer.Optimize(coarse, mismatched, 10.0, &result) &&
                     !result.qpsuccess && result.stpoints.empty(),
                 "mismatched boundary sizes should fail and clear output"))
        return 1;
    config.weight_jerk = -1.0;
    optimizer.SetConfig(config);
    if (!Require(!optimizer.Optimize(coarse, MakeArea(grid, 10.0),
                                     10.0, &result),
                 "negative weights should fail instead of being clamped"))
        return 1;
    if (!Require(!optimizer.Optimize(coarse, MakeArea(grid, 10.0),
                                     10.0, nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS speed_qp_optimizer smoke\n");
    return 0;
}
