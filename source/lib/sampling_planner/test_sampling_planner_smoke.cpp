#include "SamplingPlanner.hpp"

#include <cstdio>

namespace
{
bool Require(bool cond, const char* msg)
{
    if (!cond)
        std::fprintf(stderr, "FAIL: %s\n", msg);
    return cond;
}
}  // namespace

int main()
{
    rsim_driver::SamplingPlanner planner;
    planner.params.lateralStep = 3.5;
    planner.params.lateralRange = 3.5;
    planner.params.maxSpeed = 20.0;

    rsim_driver::FrenetState current;
    current.s = 0.0;
    current.s_d = 5.0;
    current.d = 0.0;

    rsim_driver::TrajectoryPlanResult cruise =
        planner.PlanTrajectory(current,
                               /*desiredSpeed=*/10.0,
                               /*stopS=*/rsim_driver::LARGE_NUMBER,
                               /*obstacles=*/nullptr,
                               /*numObstacles=*/0,
                               /*roadLeftBound=*/1.5,
                               /*roadRightBound=*/-1.5,
                               /*distToJunction=*/rsim_driver::LARGE_NUMBER);

    if (!Require(cruise.valid, "cruise trajectory should be valid"))
        return 1;
    if (!Require(cruise.trajectory.numPoints >= 2, "cruise trajectory should contain points"))
        return 1;
    if (!Require(cruise.trajectory.points[1].s > cruise.trajectory.points[0].s,
                 "cruise trajectory should move forward"))
        return 1;
    if (!Require(cruise.trajectory.points[cruise.trajectory.numPoints - 1].speed > current.s_d,
                 "cruise trajectory should accelerate toward desired speed"))
        return 1;

    rsim_driver::Obstacle obstacle;
    obstacle.s = 18.0;
    obstacle.d = 0.0;
    obstacle.speed = 0.0;
    obstacle.length = 5.0;
    obstacle.width = 2.0;

    rsim_driver::TrajectoryPlanResult blocked =
        planner.PlanTrajectory(current,
                               /*desiredSpeed=*/10.0,
                               /*stopS=*/rsim_driver::LARGE_NUMBER,
                               &obstacle,
                               /*numObstacles=*/1,
                               /*roadLeftBound=*/1.5,
                               /*roadRightBound=*/-1.5,
                               /*distToJunction=*/rsim_driver::LARGE_NUMBER);

    if (!Require(blocked.valid, "blocked-lane trajectory should still produce a safe plan"))
        return 1;
    const auto& end = blocked.trajectory.points[blocked.trajectory.numPoints - 1];
    if (!Require(end.s < obstacle.s, "blocked-lane trajectory should stop before obstacle"))
        return 1;

    std::fprintf(stderr, "PASS\n");
    return 0;
}
