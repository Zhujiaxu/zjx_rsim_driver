#pragma once

#include "perception/FrenetObstaclePerception.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "dynamic_programming/DpPlanner.hpp"
#include "drivable_area/DrivableArea.hpp"
#include "quadratic_programming/QpPathOptimizer.hpp"

namespace rsim_driver
{

    struct EmPlannerConfig
    {
        FrenetObstaclePerceptionConfig perception_config;
        PlanningStartConfig planning_start_config;
        DpPlannerConfig dp_config;
        DrivableAreaConfig drivable_area_config;
        QpPathOptimizerConfig qp_config;
    };

    struct EmPlannerResult
    {
        bool perception_success = false;
        bool planning_start_success = false;
        bool frenet_start_success = false;
        bool dp_success = false;
        bool drivable_area_success = false;
        bool qp_success = false;
        PlanningStartResult planning_start_result;
        CartesianFrenetState frenet_start_result;
        FrenetObstaclePerceptionResult perception_result;
        DpPlannerResult dp_result;
        DrivableArea drivable_area;
        QpPathResult qp_result;
    };

    class EmPlanner
    {
    public:
        explicit EmPlanner(const EmPlannerConfig &config = {});

        const EmPlannerConfig &config() const;
        void SetConfig(const EmPlannerConfig &config);

        // Main pipeline: actors + ego actor → DP path
        // Templated on RefPointT because both perception::Convert and
        // planning_start::ToFrenet are templated.
        template <typename RefPointT>
        bool EMPlan(const std::vector<rsim_plugin::ActorState> &actors,
                    int32_t egoActorId,
                    const rsim_plugin::ActorState &ego,
                    double currentTime,
                    const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
                    const std::vector<RefPointT> &referencePoints,
                    DpPlannerResult *result) const;

        template <typename RefPointT>
        bool EMPlanDetailed(const std::vector<rsim_plugin::ActorState> &actors,
                            int32_t egoActorId,
                            const rsim_plugin::ActorState &ego,
                            double currentTime,
                            const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
                            const std::vector<RefPointT> &referencePoints,
                            EmPlannerResult *result) const;

        const FrenetObstaclePerception &get_perception() const;
        const PlanningStart &get_planning_start() const;

    private:
        bool RunDynamicProgramming(
            const CartesianFrenetState &start,
            const std::vector<StaticFrenetObstacle> &obstacles,
            DpPlannerResult *result) const;
        bool BuildDrivableArea(
            const std::vector<DpPathPoint> &coarsePath,
            const std::vector<StaticFrenetObstacle> &obstacles,
            DrivableArea *result) const;
        bool RunQuadraticProgramming(
            const CartesianFrenetState &start,
            const std::vector<DpPathPoint> &coarsePath,
            const DrivableArea &drivableArea,
            const std::vector<StaticFrenetObstacle> &obstacles,
            QpPathResult *result) const;

        EmPlannerConfig EMconfig_;
        FrenetObstaclePerception perception_;
        PlanningStart planning_start_;
        DpPlanner dp_planner_;
        DrivableAreaBuilder drivable_area_builder_;
        QpPathOptimizer qp_path_optimizer_;
    };

    // --- template implementation ---

    template <typename RefPointT>
    bool EmPlanner::EMPlan(
        const std::vector<rsim_plugin::ActorState> &actors,
        int32_t egoActorId,
        const rsim_plugin::ActorState &ego,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
        const std::vector<RefPointT> &referencePoints,
        DpPlannerResult *result) const
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
            *result = detailedResult.dp_result;
            return false;
        }

        result->dpsuccess = detailedResult.qp_result.qpsuccess;
        result->total_cost = detailedResult.qp_result.objective;
        result->path = detailedResult.qp_result.path;
        return true;
    }

    template <typename RefPointT>
    bool EmPlanner::EMPlanDetailed(
        const std::vector<rsim_plugin::ActorState> &actors,
        int32_t egoActorId,
        const rsim_plugin::ActorState &ego,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
        const std::vector<RefPointT> &referencePoints,
        EmPlannerResult *result) const
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
                                 &output.perception_result))
        {
            *result = output;
            return false;
        }
        output.perception_success = true;

        // Step 2: PlanningStart — compute start point in Cartesian
        output.planning_start_result =
            planning_start_.Compute(ego, currentTime, previousTrajectory);
        output.planning_start_success = true;

        // Step 3: Convert start point to Frenet
        if (!planning_start_.ToFrenet(output.planning_start_result,
                                      referencePoints,
                                      &output.frenet_start_result))
        {
            *result = output;
            return false;
        }
        output.frenet_start_success = true;

        // Step 4: Dynamic Programming — plan path
        if (!RunDynamicProgramming(output.frenet_start_result,
                                   output.perception_result.static_obstacles,
                                   &output.dp_result))
        {
            *result = output;
            return false;
        }
        output.dp_success = output.dp_result.dpsuccess;

        // Step 5: DrivableArea — expand coarse DP s/l path into boundaries
        if (!BuildDrivableArea(output.dp_result.path,
                               output.perception_result.static_obstacles,
                               &output.drivable_area))
        {
            *result = output;
            return false;
        }
        output.drivable_area_success = true;

        // Step 6: QuadraticProgramming — smooth DP path inside drivable area
        if (!RunQuadraticProgramming(output.frenet_start_result,
                                     output.dp_result.path,
                                     output.drivable_area,
                                     output.perception_result.static_obstacles,
                                     &output.qp_result))
        {
            *result = output;
            return false;
        }
        output.qp_success = output.qp_result.qpsuccess;

        *result = output;
        return output.dp_success &&
               output.drivable_area_success &&
               output.qp_success;
    }

} // namespace rsim_driver
