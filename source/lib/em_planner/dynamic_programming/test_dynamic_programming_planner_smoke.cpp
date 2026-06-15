#include "dynamic_programming/DpPlanner.hpp"

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

rsim_driver::CartesianFrenetState MakeStart()
{
    rsim_driver::CartesianFrenetState start;
    start.s = 0.0;
    start.l = 0.0;
    start.l_prime = 0.0;
    start.l_double_prime = 0.0;
    return start;
}

rsim_driver::DpPlannerConfig MakeConfig()
{
    rsim_driver::DpPlannerConfig config;
    config.s_step = 0.5;
    config.s_step_count = 2;
    config.l_step = 0.4;
    config.left_l_step_count = 1;
    config.right_l_step_count = 1;
    config.weight_l_prime = 1.0;
    config.weight_l_double_prime = 1.0;
    config.weight_ref_l = 1.0;
    config.weight_collision = 10.0;
    config.collision.collision_distance = 0.2;
    config.collision.risk_distance = 0.3;
    return config;
}

}  // namespace

int main()
{
    rsim_driver::DpPlannerResult result;
    rsim_driver::DpPlannerConfig config = MakeConfig();
    rsim_driver::DpPlanner planner(config);

    if (!Require(planner.Plan(MakeStart(), {}, &result) &&
                     result.dpsuccess,
                 "planner should succeed without obstacles"))
        return 1;
    if (!Require(result.path.size() == 3 &&
                 Near(result.path[0].s, 0.0) &&
                 Near(result.path[1].s, 0.5) &&
                 Near(result.path[2].s, 1.0),
                 "planner should output sampled s path"))
        return 1;
    for (const rsim_driver::DpPathPoint& point : result.path)
    {
        if (!Require(Near(point.l, 0.0),
                     "unblocked path should stay on reference line"))
            return 1;
        if (!Require(Near(point.l_prime, 0.0) &&
                         Near(point.l_double_prime, 0.0),
                     "default path derivatives should remain zero"))
            return 1;
    }

    rsim_driver::CartesianFrenetState derivativeStart = MakeStart();
    derivativeStart.l_prime = 0.2;
    derivativeStart.l_double_prime = -0.1;
    rsim_driver::DpPlannerConfig derivativeConfig = MakeConfig();
    derivativeConfig.left_l_step_count = 0;
    derivativeConfig.right_l_step_count = 0;
    derivativeConfig.s_step_count = 1;
    planner.SetConfig(derivativeConfig);
    if (!Require(planner.Plan(derivativeStart, {}, &result) &&
                     result.dpsuccess,
                 "planner should preserve derivative start state"))
        return 1;
    if (!Require(result.path.size() == 2 &&
                     Near(result.path.front().l_prime, derivativeStart.l_prime) &&
                     Near(result.path.front().l_double_prime,
                          derivativeStart.l_double_prime),
                 "first output point should keep start derivatives"))
        return 1;
    if (!Require(Near(result.path.back().l_prime, 0.0) &&
                     Near(result.path.back().l_double_prime, 0.0),
                 "lattice output point derivatives should follow end boundary"))
        return 1;

    const std::vector<rsim_driver::StaticFrenetObstacle> centerObstacle = {
        {1, 0.5, 0.0},
    };
    planner.SetConfig(config);
    if (!Require(planner.Plan(MakeStart(), centerObstacle, &result) &&
                     result.dpsuccess,
                 "planner should find side path around center obstacle"))
        return 1;
    if (!Require(result.path.size() == 3 &&
                 std::fabs(result.path[1].l) >= 0.4 - 1e-9,
                 "blocked center node should force side lattice point"))
        return 1;

    rsim_driver::CartesianFrenetState shiftedStart = MakeStart();
    shiftedStart.l = 1.2;
    const std::vector<rsim_driver::StaticFrenetObstacle> shiftedCenterObstacle = {
        {2, 0.5, 1.2},
    };
    planner.SetConfig(MakeConfig());
    if (!Require(planner.Plan(shiftedStart,
                              shiftedCenterObstacle,
                              &result) &&
                     result.dpsuccess,
                 "planner should sample lateral lattice around shifted start"))
        return 1;
    if (!Require(result.path.size() == 3 &&
                     (Near(result.path[1].l, 0.8) ||
                      Near(result.path[1].l, 1.6)),
                 "shifted start obstacle should force adjacent shifted lattice point"))
        return 1;

    config.left_l_step_count = 0;
    config.right_l_step_count = 0;
    config.s_step_count = 1;
    planner.SetConfig(config);
    if (!Require(!planner.Plan(MakeStart(), centerObstacle, &result) &&
                     !result.dpsuccess &&
                     result.path.empty(),
                 "planner should fail when every target node collides"))
        return 1;

    std::fprintf(stderr, "PASS dynamic_programming_planner smoke\n");
    return 0;
}
