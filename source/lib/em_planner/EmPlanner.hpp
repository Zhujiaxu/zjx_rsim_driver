#pragma once

#include "perception/FrenetObstaclePerception.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "dynamic_programming/DpPlanner.hpp"

namespace rsim_driver
{

struct EmPlannerConfig
{
    FrenetObstaclePerceptionConfig perception;
    PlanningStartConfig planning_start;
    DpPlannerConfig dp;
};

struct EmPlannerResult
{
    bool perception_success = false;
    bool planning_start_success = false;
    bool frenet_start_success = false;
    bool dp_success = false;
    PlanningStartResult planning_start;
    CartesianFrenetState frenet_start;
    FrenetObstaclePerceptionResult perception;
    DpPlannerResult dp;
};

class EmPlanner
{
public:
    explicit EmPlanner(const EmPlannerConfig& config = {});

    const EmPlannerConfig& config() const;
    void SetConfig(const EmPlannerConfig& config);

    // Main pipeline: actors + ego actor → DP path
    // Templated on RefPointT because both perception::Convert and
    // planning_start::ToFrenet are templated.
    template <typename RefPointT>
    bool EMPlan(const std::vector<rsim_plugin::ActorState>& actors,
              int32_t egoActorId,
              const rsim_plugin::ActorState& ego,
              double currentTime,
              const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
              const std::vector<RefPointT>& referencePoints,
              DpPlannerResult* result) const;

    template <typename RefPointT>
    bool EMPlanDetailed(const std::vector<rsim_plugin::ActorState>& actors,
                        int32_t egoActorId,
                        const rsim_plugin::ActorState& ego,
                        double currentTime,
                        const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
                        const std::vector<RefPointT>& referencePoints,
                        EmPlannerResult* result) const;

    const FrenetObstaclePerception& perception() const;
    const PlanningStart& planning_start() const;

private:
    EmPlannerConfig EMconfig_;
    FrenetObstaclePerception perception_;
    PlanningStart planning_start_;
};

// --- template implementation ---

template <typename RefPointT>
bool EmPlanner::EMPlan(
    const std::vector<rsim_plugin::ActorState>& actors,
    int32_t egoActorId,
    const rsim_plugin::ActorState& ego,
    double currentTime,
    const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
    const std::vector<RefPointT>& referencePoints,
    DpPlannerResult* result) const
{
    if (result == nullptr)
        return false;

    EmPlannerResult detailedResult;
    if (!EMPlanDetailed(actors,
                        egoActorId,
                        ego,
                        currentTime,
                        previousTrajectory,
                        referencePoints,
                        &detailedResult))
    {
        *result = detailedResult.dp;
        return false;
    }

    *result = detailedResult.dp;
    return true;
}

template <typename RefPointT>
bool EmPlanner::EMPlanDetailed(
    const std::vector<rsim_plugin::ActorState>& actors,
    int32_t egoActorId,
    const rsim_plugin::ActorState& ego,
    double currentTime,
    const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
    const std::vector<RefPointT>& referencePoints,
    EmPlannerResult* result) const
{
    if (result == nullptr)
        return false;

    EmPlannerResult output;
    if (referencePoints.empty())
    {
        *result = output;
        return false;
    }

    // Step 1: Perception — convert actors to Frenet obstacles
    if (!perception_.Convert(actors,
                             egoActorId,
                             referencePoints,
                             &output.perception))
    {
        *result = output;
        return false;
    }
    output.perception_success = true;

    // Step 2: PlanningStart — compute start point in Cartesian
    output.planning_start =
        planning_start_.Compute(ego, currentTime, previousTrajectory);
    output.planning_start_success = true;

    // Step 3: Convert start point to Frenet
    if (!planning_start_.ToFrenet(output.planning_start,
                                  referencePoints,
                                  &output.frenet_start))
    {
        *result = output;
        return false;
    }
    output.frenet_start_success = true;

    // Step 4: Dynamic Programming — plan path
    if (!rsim_driver::Plan(output.frenet_start,
                           output.perception.static_obstacles,
                           EMconfig_.dp,
                           &output.dp))
    {
        *result = output;
        return false;
    }
    output.dp_success = output.dp.success;

    *result = output;
    return output.dp_success;
}

}  // namespace rsim_driver
