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
    config.total_length = 1.0;
    config.l_step = 0.4;
    config.left_width = 0.4;
    config.right_width = 0.4;
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

    if (!Require(rsim_driver::Plan(MakeStart(), {}, config, &result) &&
                     result.success,
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
    }

    const std::vector<rsim_driver::StaticFrenetObstacle> centerObstacle = {
        {1, 0.5, 0.0},
    };
    if (!Require(rsim_driver::Plan(MakeStart(), centerObstacle, config, &result) &&
                     result.success,
                 "planner should find side path around center obstacle"))
        return 1;
    if (!Require(result.path.size() == 3 &&
                 std::fabs(result.path[1].l) >= 0.4 - 1e-9,
                 "blocked center node should force side lattice point"))
        return 1;

    config.left_width = 0.0;
    config.right_width = 0.0;
    config.total_length = 0.5;
    if (!Require(!rsim_driver::Plan(MakeStart(), centerObstacle, config, &result) &&
                     !result.success &&
                     result.path.empty(),
                 "planner should fail when every target node collides"))
        return 1;

    std::fprintf(stderr, "PASS dynamic_programming_planner smoke\n");
    return 0;
}
