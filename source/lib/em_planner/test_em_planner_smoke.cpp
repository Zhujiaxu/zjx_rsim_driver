#include "EMPlanner.hpp"
#include "IPlanner.hpp"
#include "ReferenceLine.hpp"

#include <cmath>
#include <cstdio>

namespace
{
bool Require(bool cond, const char* msg)
{
    if (!cond) std::fprintf(stderr, "FAIL: %s\n", msg);
    return cond;
}

rsim_driver::EMPlannerInput BaseInput()
{
    rsim_driver::EMPlannerInput input;
    input.ego.s = 0.0;
    input.ego.s_d = 5.0;
    input.ego.d = 0.0;
    input.desiredSpeed = 10.0;
    input.horizonTime = 5.0;
    input.reference.laneWidth = 3.5;
    input.reference.leftBound = 1.5;
    input.reference.rightBound = -1.5;
    return input;
}

// Build a straight reference line along the x-axis.
rsim_driver::ReferenceLine BuildStraightLine(int numPoints = 50, double step = 2.0)
{
    rsim_driver::ReferenceLine rl;
    for (int i = 0; i < numPoints; ++i)
    {
        rsim_driver::ReferencePoint pt;
        pt.x   = step * static_cast<double>(i);
        pt.y   = 0.0;
        pt.hdg = 0.0;
        pt.k   = 0.0;
        pt.dk  = 0.0;
        pt.s   = step * static_cast<double>(i);
        rl.points.push_back(pt);
    }
    return rl;
}
}  // namespace

int main()
{
    rsim_driver::EMPlanner planner;
    planner.params.maxSpeed = 20.0;
    planner.params.maxAccel = 4.0;
    planner.params.maxDecel = 6.0;

    // --- Legacy path tests (no reference line) ---

    rsim_driver::EMPlannerInput cruiseInput = BaseInput();
    rsim_driver::EMPlannerOutput cruise = planner.Plan(cruiseInput);
    if (!Require(cruise.valid, "cruise trajectory should be valid")) return 1;
    if (!Require(cruise.trajectory.numPoints >= 2, "cruise trajectory should contain points")) return 1;
    if (!Require(cruise.trajectory.points[1].s > cruise.trajectory.points[0].s,
                 "cruise trajectory should move forward")) return 1;
    if (!Require(cruise.speedDecision.type == rsim_driver::SpeedDecisionType::CRUISE,
                 "cruise should choose CRUISE speed decision")) return 1;

    rsim_driver::EMPlannerInput emptyObstacleInput = BaseInput();
    emptyObstacleInput.obstacles = nullptr;
    emptyObstacleInput.numObstacles = 4;
    rsim_driver::EMPlannerOutput emptyObstacle = planner.Plan(emptyObstacleInput);
    if (!Require(emptyObstacle.valid, "null obstacle pointer should be treated as no obstacles")) return 1;

    rsim_driver::Obstacle slowLead;
    slowLead.s = 25.0; slowLead.d = 0.0; slowLead.speed = 3.0;
    slowLead.length = 5.0; slowLead.width = 2.0;

    rsim_driver::EMPlannerInput followInput = BaseInput();
    followInput.obstacles = &slowLead; followInput.numObstacles = 1;
    rsim_driver::EMPlannerOutput follow = planner.Plan(followInput);
    if (!Require(follow.valid, "follow trajectory should be valid")) return 1;
    if (!Require(follow.speedDecision.type == rsim_driver::SpeedDecisionType::FOLLOW
                     || follow.speedDecision.type == rsim_driver::SpeedDecisionType::STOP,
                 "slow lead should choose FOLLOW or STOP")) return 1;

    rsim_driver::Obstacle closeBlock;
    closeBlock.s = 18.0; closeBlock.d = 0.0; closeBlock.speed = 0.0;
    closeBlock.length = 5.0; closeBlock.width = 2.0;

    rsim_driver::EMPlannerInput blockedInput = BaseInput();
    blockedInput.obstacles = &closeBlock; blockedInput.numObstacles = 1;
    rsim_driver::EMPlannerOutput blocked = planner.Plan(blockedInput);
    if (!Require(blocked.valid, "blocked lane should still produce a safe trajectory")) return 1;
    if (!Require(blocked.speedDecision.type == rsim_driver::SpeedDecisionType::STOP,
                 "blocked lane should choose STOP")) return 1;

    rsim_driver::EMPlannerInput clampInput = BaseInput();
    clampInput.ego.d = 1.3;
    clampInput.reference.leftBound = 0.5;
    clampInput.reference.rightBound = -0.5;
    rsim_driver::EMPlannerOutput clamped = planner.Plan(clampInput);
    if (!Require(clamped.valid, "clamped lateral bounds should still produce a trajectory")) return 1;
    for (int i = 0; i < clamped.trajectory.numPoints; ++i)
    {
        if (!Require(clamped.trajectory.points[i].d <= clampInput.reference.leftBound + 1e-6
                         && clamped.trajectory.points[i].d >= clampInput.reference.rightBound - 1e-6,
                     "trajectory d should stay inside lateral bounds")) return 1;
    }

    rsim_driver::EMPlanner limitedPlanner = planner;
    limitedPlanner.params.maxAccel = 0.01;
    rsim_driver::EMPlannerInput infeasibleInput = BaseInput();
    infeasibleInput.ego.s_d = 0.0;
    infeasibleInput.desiredSpeed = 20.0;
    rsim_driver::EMPlannerOutput infeasible = limitedPlanner.Plan(infeasibleInput);
    if (!Require(!infeasible.valid, "infeasible dynamic limits should fail planning")) return 1;
    if (!Require(infeasible.debug.failureReason[0] != '\0',
                 "failed planning should include a failure reason")) return 1;

    // --- DP+QP pipeline tests (with reference line) ---

    rsim_driver::ReferenceLine straightLine = BuildStraightLine();

    // Test 1: Cruise with reference line via IPlanner interface.
    {
        rsim_driver::PlannerInput in;
        in.ego.s = 0.0; in.ego.s_d = 10.0; in.ego.d = 0.0;
        in.desiredSpeed = 10.0;
        in.horizonTime = 5.0;
        in.referenceLine = &straightLine;
        in.obstacles = nullptr;
        in.numObstacles = 0;

        rsim_driver::PlannerOutput out = planner.Plan(in);
        if (!Require(out.valid, "DP+QP: cruise should be valid")) return 1;
        if (!Require(out.trajectory.numPoints >= 2, "DP+QP: should have trajectory points")) return 1;
        if (!Require(out.frenet.valid, "DP+QP: frenet trajectory should be valid")) return 1;
    }

    // Test 2: Slow lead vehicle with reference line.
    {
        rsim_driver::PlannerInput in;
        in.ego.s = 0.0; in.ego.s_d = 15.0; in.ego.d = 0.0;
        in.desiredSpeed = 15.0;
        in.horizonTime = 5.0;
        in.referenceLine = &straightLine;
        in.obstacles = &slowLead;
        in.numObstacles = 1;

        rsim_driver::PlannerOutput out = planner.Plan(in);
        if (!Require(out.valid, "DP+QP: follow scenario should be valid")) return 1;
        if (!Require(out.trajectory.numPoints >= 2, "DP+QP: follow should have points")) return 1;
    }

    // Test 3: IPlanner interface available.
    {
        rsim_driver::IPlanner* iplanner = &planner;
        rsim_driver::PlannerInput in;
        in.ego.s = 0.0; in.ego.s_d = 8.0; in.ego.d = 0.0;
        in.desiredSpeed = 8.0;
        in.horizonTime = 5.0;
        in.referenceLine = &straightLine;

        rsim_driver::PlannerOutput out = iplanner->Plan(in);
        if (!Require(out.valid, "DP+QP via IPlanner*: should be valid")) return 1;
    }

    std::fprintf(stderr, "PASS\n");
    return 0;
}
