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

bool Near(double actual, double expected, double tolerance = 1e-6)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DynamicPlanSpeedPoint MakeStart(double s = 0.0, double v = 13.0, double a = 0.0)
{
    rsim_driver::DynamicPlanSpeedPoint pt;
    pt.t = 0.0;
    pt.s = s;
    pt.v = v;
    pt.a = a;
    return pt;
}

rsim_driver::StDrivableArea MakeFullDrivableArea(int num_points, double dt, double ref_len)
{
    rsim_driver::StDrivableArea area;
    area.lower_boundary.reserve(static_cast<std::size_t>(num_points));
    area.upper_boundary.reserve(static_cast<std::size_t>(num_points));
    for (int i = 0; i < num_points; ++i)
    {
        const double t = static_cast<double>(i) * dt;
        area.lower_boundary.push_back({0.0, t});
        area.upper_boundary.push_back({ref_len, t});
    }
    return area;
}

bool CheckTaylor3rdOrder(const rsim_driver::DynamicPlanSpeedPoint &prev,
                         const rsim_driver::DynamicPlanSpeedPoint &curr,
                         double dt,
                         double tolerance = 1e-4)
{
    // s_{i+1} - s_i - dt*v_i - dt²/3*a_i - dt²/6*a_{i+1} = 0
    const double residual = curr.s - prev.s - dt * prev.v -
                            (dt * dt / 3.0) * prev.a -
                            (dt * dt / 6.0) * curr.a;
    return std::fabs(residual) <= tolerance;
}

bool CheckTrapezoidalV(const rsim_driver::DynamicPlanSpeedPoint &prev,
                       const rsim_driver::DynamicPlanSpeedPoint &curr,
                       double dt,
                       double tolerance = 1e-4)
{
    // v_{i+1} - v_i - dt/2*a_i - dt/2*a_{i+1} = 0
    const double residual = curr.v - prev.v -
                            (dt / 2.0) * prev.a -
                            (dt / 2.0) * curr.a;
    return std::fabs(residual) <= tolerance;
}

rsim_driver::SpeedQpOptimizerConfig MakeConfig(int num_points = 9, double dt = 1.0)
{
    rsim_driver::SpeedQpOptimizerConfig config;
    config.num_points = num_points;
    config.dt = dt;
    config.longitudinal_safety_buffer = 0.0;  // full-open tests have no obstacles
    return config;
}

}  // namespace

int main()
{
    // --- Test 1: Constant speed cruise ---
    {
        const double ref_speed = 13.0;
        auto config = MakeConfig();
        config.reference_speed = ref_speed;
        config.weight_reference_speed = 10.0;
        config.weight_acceleration = 10.0;
        config.weight_jerk = 1.0;
        config.weight_progress = 0.1;

        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart(0.0, ref_speed, 0.0);
        const double ref_len = 120.0;
        auto area = MakeFullDrivableArea(config.num_points, config.dt, ref_len);

        if (!Require(optimizer.Optimize(start, area, ref_len, &result),
                     "cruise should solve"))
            return 1;

        if (!Require(result.qpsuccess, "cruise qpsuccess should be true"))
            return 1;

        if (!Require(static_cast<int>(result.stpoints.size()) == config.num_points,
                     "cruise should have num_points entries"))
            return 1;

        // Verify start state
        if (!Require(Near(result.stpoints[0].s, start.s) &&
                         Near(result.stpoints[0].v, start.v) &&
                         Near(result.stpoints[0].a, start.a),
                     "cruise start state should match"))
            return 1;

        // Verify speeds close to reference (allow some deviation due to jerk)
        for (const auto &pt : result.stpoints)
        {
            if (!Require(Near(pt.v, ref_speed, 1.0),
                         "cruise speed should be near reference"))
                return 1;
            if (!Require(Near(pt.a, 0.0, 1.0),
                         "cruise acceleration should be near 0"))
                return 1;
        }

        // Verify kinematic constraints
        for (std::size_t i = 1; i < result.stpoints.size(); ++i)
        {
            if (!Require(CheckTaylor3rdOrder(result.stpoints[i - 1],
                                              result.stpoints[i], config.dt),
                         "cruise should satisfy Taylor constraint"))
                return 1;
            if (!Require(CheckTrapezoidalV(result.stpoints[i - 1],
                                            result.stpoints[i], config.dt),
                         "cruise should satisfy trapezoidal v constraint"))
                return 1;
        }

        // Verify s increases monotonically
        for (std::size_t i = 1; i < result.stpoints.size(); ++i)
        {
            if (!Require(result.stpoints[i].s >= result.stpoints[i - 1].s - 1e-9,
                         "cruise s should be non-decreasing"))
                return 1;
        }

        // Verify acceleration bounds
        for (const auto &pt : result.stpoints)
        {
            if (!Require(pt.a >= config.a_min - 1e-9 && pt.a <= config.a_max + 1e-9,
                         "cruise acceleration should be within bounds"))
                return 1;
        }
    }

    // --- Test 2: Acceleration from stop ---
    {
        auto config = MakeConfig();
        config.reference_speed = 13.0;
        config.weight_reference_speed = 5.0;
        config.weight_acceleration = 5.0;
        config.weight_jerk = 1.0;

        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart(0.0, 0.0, 0.0);
        const double ref_len = 120.0;
        auto area = MakeFullDrivableArea(config.num_points, config.dt, ref_len);

        if (!Require(optimizer.Optimize(start, area, ref_len, &result),
                     "accel from stop should solve"))
            return 1;

        // Speeds should be non-negative and approach reference
        double max_speed = 0.0;
        for (const auto &pt : result.stpoints)
        {
            if (!Require(pt.v >= -1e-9,
                         "accel speed should be non-negative"))
                return 1;
            if (pt.v > max_speed)
                max_speed = pt.v;
        }

        // Max speed should approach reference
        if (!Require(max_speed > 0.5 * config.reference_speed,
                     "accel max speed should approach reference"))
            return 1;

        // Verify kinematic constraints
        for (std::size_t i = 1; i < result.stpoints.size(); ++i)
        {
            if (!Require(CheckTaylor3rdOrder(result.stpoints[i - 1],
                                              result.stpoints[i], config.dt),
                         "accel should satisfy Taylor constraint"))
                return 1;
            if (!Require(CheckTrapezoidalV(result.stpoints[i - 1],
                                            result.stpoints[i], config.dt),
                         "accel should satisfy trapezoidal v constraint"))
                return 1;
        }
    }

    // --- Test 3: ST drivable area constraint ---
    {
        auto config = MakeConfig();
        config.reference_speed = 13.0;
        config.weight_reference_speed = 5.0;
        config.weight_acceleration = 5.0;
        config.weight_jerk = 1.0;
        config.longitudinal_safety_buffer = 6.0;  // restore obstacle buffer for this test
        // ego_length=4.0 → totalMargin = 2.0+6.0 = 8.0

        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart(0.0, 13.0, 0.0);
        const double ref_len = 120.0;

        // Narrow drivable area: upper clamped to 40 at t>=2, forcing slow-down
        // With totalMargin=8: upper bound for s_i ≤ upper[i].s - 8
        // So at t=2: s must be ≤ 40 - 8 = 32
        auto area = MakeFullDrivableArea(config.num_points, config.dt, ref_len);
        for (int i = 0; i < config.num_points; ++i)
        {
            const double t = static_cast<double>(i) * config.dt;
            if (t >= 2.0)
                area.upper_boundary[static_cast<std::size_t>(i)].s = 40.0;
        }

        if (!Require(optimizer.Optimize(start, area, ref_len, &result),
                     "drivable area constraint should solve"))
            return 1;

        // Verify s values are within drivable area (with conditional ego margin, matching implementation)
        const double totalMargin = 0.5 * config.ego_length + config.longitudinal_safety_buffer;
        for (const auto &pt : result.stpoints)
        {
            int idx = static_cast<int>(pt.t / config.dt + 0.5);
            if (idx < 0) idx = 0;
            if (idx >= config.num_points) idx = config.num_points - 1;
            const double raw_lb = area.lower_boundary[static_cast<std::size_t>(idx)].s;
            const double raw_ub = area.upper_boundary[static_cast<std::size_t>(idx)].s;
            // Margin only applied to non-open-road bounds (matching SpeedQpOptimizer logic)
            const double lb = (raw_lb > 1e-9) ? raw_lb + totalMargin : raw_lb;
            const double ub = (raw_ub < ref_len - 1e-9) ? raw_ub - totalMargin : raw_ub;
            if (!Require(pt.s >= lb - 1e-6 && pt.s <= ub + 1e-6,
                         "s must be within drivable area bounds (with ego margin)"))
                return 1;
        }
    }

    // --- Test 4: Acceleration bounds ---
    {
        auto config = MakeConfig();
        config.a_min = -1.0;
        config.a_max = 1.0;
        config.reference_speed = 13.0;
        config.weight_reference_speed = 5.0;
        config.weight_acceleration = 1.0;
        config.weight_jerk = 1.0;

        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart(0.0, 0.0, 0.0);
        const double ref_len = 120.0;
        auto area = MakeFullDrivableArea(config.num_points, config.dt, ref_len);

        if (!Require(optimizer.Optimize(start, area, ref_len, &result),
                     "accel bounds should solve"))
            return 1;

        for (const auto &pt : result.stpoints)
        {
            if (!Require(pt.a >= config.a_min - 1e-6 && pt.a <= config.a_max + 1e-6,
                         "acceleration should be within tight bounds"))
                return 1;
        }
    }

    // --- Test 5: Progress cost ---
    {
        auto config = MakeConfig();
        config.reference_speed = 13.0;
        config.weight_reference_speed = 0.0;
        config.weight_progress = 100.0;
        config.weight_acceleration = 1.0;
        config.weight_jerk = 1.0;
        config.longitudinal_safety_buffer = 0.0;

        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart(0.0, 13.0, 0.0);
        const double ref_len = 100.0;
        auto area = MakeFullDrivableArea(config.num_points, config.dt, ref_len);

        if (!Require(optimizer.Optimize(start, area, ref_len, &result),
                     "progress cost should solve"))
            return 1;

        // With high progress weight, s_{N-1} should be close to ref_len
        if (!Require(Near(result.stpoints.back().s, ref_len, 5.0),
                     "final s should be near target with high progress weight"))
            return 1;
    }

    // --- Test 6: Null result ---
    {
        auto config = MakeConfig();
        rsim_driver::SpeedQpOptimizer optimizer(config);
        auto start = MakeStart();
        auto area = MakeFullDrivableArea(config.num_points, config.dt, 100.0);
        if (!Require(!optimizer.Optimize(start, area, 100.0, nullptr),
                     "null result should fail"))
            return 1;
    }

    // --- Test 7: Invalid config ---
    {
        auto config = MakeConfig();
        config.dt = 0.0;
        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart();
        auto area = MakeFullDrivableArea(config.num_points, 1.0, 100.0);
        if (!Require(!optimizer.Optimize(start, area, 100.0, &result),
                     "zero dt should fail"))
            return 1;
        if (!Require(!result.qpsuccess && result.stpoints.empty(),
                     "failed result should be empty"))
            return 1;
    }

    // --- Test 8: Mismatched boundary size ---
    {
        auto config = MakeConfig(9, 1.0);
        rsim_driver::SpeedQpOptimizer optimizer(config);
        rsim_driver::SpeedQpResult result;
        auto start = MakeStart();
        auto area = MakeFullDrivableArea(5, config.dt, 100.0);  // only 5 points, config expects 9
        if (!Require(!optimizer.Optimize(start, area, 100.0, &result),
                     "mismatched boundary size should fail"))
            return 1;
    }

    std::fprintf(stderr, "PASS speed_qp_optimizer smoke\n");
    return 0;
}
