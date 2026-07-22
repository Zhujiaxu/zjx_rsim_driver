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

bool Near(double actual, double expected, double tolerance = 2e-5)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::QpSpeedOptimizerConfig MakeConfig()
{
    rsim_driver::QpSpeedOptimizerConfig config;
    config.dt = 0.2;
    config.dqpt = 0.2;
    config.reference_speed = 1.0;
    config.ego_length = 0.2;
    config.longitudinal_safety_buffer = 0.0;
    config.weight_reference_speed = 5.0;
    config.weight_acceleration = 2.0;
    config.weight_jerk = 1.0;
    config.weight_progress = 2.0;
    config.a_min = -5.0;
    config.a_max = 3.0;
    return config;
}

rsim_driver::StDrivableAreaResult MakeArea(double horizon,
                                            double lower,
                                            double upper)
{
    rsim_driver::StDrivableAreaResult area;
    area.Flag = rsim_driver::StDrivableAreaFallback::Success;
    for (double t = 0.0; t <= horizon + 1e-9; t += 0.2)
    {
        area.lower_boundary.push_back({lower, t});
        area.upper_boundary.push_back({upper, t});
    }
    return area;
}

bool CheckKinematics(
    const std::vector<rsim_driver::DynamicPlanSpeedPoint> &points)
{
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const auto &previous = points[i - 1];
        const auto &current = points[i];
        const double dt = current.t - previous.t;
        const double positionResidual =
            current.s - previous.s - dt * previous.v -
            dt * dt * previous.a / 3.0 -
            dt * dt * current.a / 6.0;
        const double velocityResidual =
            current.v - previous.v - dt * previous.a / 2.0 -
            dt * current.a / 2.0;
        if (!Near(positionResidual, 0.0) ||
            !Near(velocityResidual, 0.0))
        {
            return false;
        }
    }
    return true;
}

bool CheckNoReverse(
    const std::vector<rsim_driver::DynamicPlanSpeedPoint> &points)
{
    constexpr double kConstraintTolerance = 2e-5;
    if (points.size() < 2)
        return false;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        if (!std::isfinite(points[i].s) || !std::isfinite(points[i].v) ||
            points[i].v < -kConstraintTolerance ||
            (i > 0 && points[i].s + kConstraintTolerance < points[i - 1].s))
        {
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    const rsim_driver::QpSpeedOptimizerConfig config = MakeConfig();
    rsim_driver::SpeedQpOptimizer optimizer(config);
    rsim_driver::QpSpeedOptimizerResult result;

    const auto forwardArea = MakeArea(1.0, -0.1, 10.1);
    const rsim_driver::DynamicPlanSpeedPoint forwardStart{0.0, 0.0, 1.0, 0.0};
    const bool forwardSolved =
        optimizer.Optimize(forwardStart, forwardArea, &result);
    if (!forwardSolved)
    {
        std::fprintf(stderr, "forward fallback=%d points=%zu\n",
                     static_cast<int>(result.Flag), result.stpoints.size());
    }
    if (!Require(forwardSolved &&
                     result.Flag ==
                         rsim_driver::QpSpeedOptimizerFallback::Success &&
                     result.stpoints.size() == 6U,
                 "forward speed QP should solve"))
    {
        return 1;
    }
    if (!Require(Near(result.stpoints.front().s, forwardStart.s) &&
                     Near(result.stpoints.front().v, forwardStart.v) &&
                     Near(result.stpoints.front().a, forwardStart.a),
                 "QP should satisfy the start-state equality"))
    {
        return 1;
    }
    if (!Require(CheckNoReverse(result.stpoints),
                 "QP nodes should never reverse") ||
        !Require(CheckKinematics(result.stpoints),
                 "QP nodes should satisfy the kinematic equalities"))
    {
        return 1;
    }

    rsim_driver::QpSpeedOptimizerConfig stopConfig = config;
    stopConfig.reference_speed = 0.0;
    stopConfig.weight_progress = 0.0;
    rsim_driver::SpeedQpOptimizer stopOptimizer(stopConfig);
    const rsim_driver::DynamicPlanSpeedPoint stopStart{0.0, 0.0, 0.0, 0.0};
    if (!Require(stopOptimizer.Optimize(stopStart, forwardArea, &result),
                 "zero-speed stop should remain feasible"))
    {
        return 1;
    }
    for (const auto &point : result.stpoints)
    {
        if (!Require(Near(point.s, 0.0) && Near(point.v, 0.0),
                     "stop solution should remain stationary within solver tolerance"))
        {
            return 1;
        }
    }

    const rsim_driver::DynamicPlanSpeedPoint movingStart{0.0, 0.0, 1.0, 0.0};
    const auto pinnedStopArea = MakeArea(1.0, -0.1, 0.1);
    if (!Require(!optimizer.Optimize(movingStart, pinnedStopArea, &result) &&
                     result.stpoints.empty(),
                 "a corridor requiring backward motion should be infeasible"))
    {
        return 1;
    }

    if (!Require(!optimizer.Optimize(forwardStart, forwardArea, nullptr),
                 "null output should fail"))
    {
        return 1;
    }

    std::fprintf(stderr, "PASS speed_qp_optimizer smoke\n");
    return 0;
}
