#pragma once

#include "perception/FrenetObstaclePerception.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "local_path_planning/drivable_area/DrivableArea.hpp"
#include "local_path_planning/dynamic_programming/DpPlanner.hpp"
#include "local_path_planning/quadratic_programming/QpPathOptimizer.hpp"
#include "local_speed_planning/dynamic_speed_planning/DynamicSpeedPlanner.hpp"

namespace rsim_driver
{

    struct EmPlannerConfig
    {
        FrenetObstaclePerceptionConfig perception_config;
        PlanningStartConfig planning_start_config;
        DpPlannerConfig dp_config;
        DrivableAreaConfig drivable_area_config;
        QpPathOptimizerConfig qp_config;
        DynamicPlanSpeedConfig dpspeed_config;
    };

    struct EmTrajectoryPoint
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double k = 0.0;
        double v = 0.0;
        double a = 0.0;
        double time = 0.0;
    };

    struct EmPlannerResult
    {

        bool perception_success = false;
        bool static_perception_success = false;
        bool dynamic_perception_success = false;
        bool planning_start_success = false;
        bool frenet_start_success = false;
        bool dp_success = false;
        bool drivable_area_success = false;
        bool qp_success = false;
        bool speed_success = false;
        bool trajectory_success = false;
        PlanningStartResult planning_start_result;
        CartesianFrenetState frenet_start_result;
        StaticFrenetObstaclePerceptionResult static_perception_result;
        DynamicFrenetObstaclePerceptionResult dynamic_perception_result;
        DpPlannerResult dp_result;
        DrivableArea drivable_area;
        std::vector<DpPathPoint> localfrenetpath;
        QpPathResult qp_result;
        localreferencelinepath speed_reference_line;
        DynamicPlanSpeedResult speed_result;
        std::vector<EmTrajectoryPoint> trajectory;
    };

    class EmPlanner
    {
    public:
        explicit EmPlanner(const EmPlannerConfig &config = {});

        const EmPlannerConfig &config() const;
        void SetConfig(const EmPlannerConfig &config);

        // Main pipeline: actors + ego actor → DP path
        // Templated on RefPointT because perception conversion and
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
        bool RunDynamicSpeedPlanning(
            const PlanningStartResult &start,
            const localreferencelinepath &referenceLine,
            const DynamicFrenetObstaclePerceptionResult &dynamicObstacles,
            DynamicPlanSpeedResult *result) const;
        bool BuildTrajectory(
            const localreferencelinepath &referenceLine,
            const DynamicPlanSpeedResult &speedResult,
            double absoluteStartTime,
            std::vector<EmTrajectoryPoint> *result) const;
        template <typename RefPointT>
        bool RunQuadraticProgramming(
            const CartesianFrenetState &start,
            const std::vector<DpPathPoint> &coarsePath,
            const DrivableArea &drivableArea,
            const std::vector<StaticFrenetObstacle> &obstacles,
            const std::vector<RefPointT> &referencePoints,
            QpPathResult *result) const;

        EmPlannerConfig EMconfig_;
        FrenetObstaclePerception perception_;
        PlanningStart planning_start_;
        DpPlanner dp_planner_;
        DrivableAreaBuilder drivable_area_builder_;
        QpPathOptimizer qp_path_optimizer_;
        DynamicPlanSpeedPlanner speed_planner_;
        mutable std::vector<DpPathPoint> localfrenetpath_;
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
        result->path = detailedResult.localfrenetpath;
        return true;
    }

    template <typename RefPointT>
    bool EmPlanner::RunQuadraticProgramming(
        const CartesianFrenetState &start,
        const std::vector<DpPathPoint> &coarsePath,
        const DrivableArea &drivableArea,
        const std::vector<StaticFrenetObstacle> &obstacles,
        const std::vector<RefPointT> &referencePoints,
        QpPathResult *result) const
    {
        return qp_path_optimizer_.Optimize(start,
                                           coarsePath,
                                           drivableArea,
                                           obstacles,
                                           referencePoints,
                                           &localfrenetpath_,
                                           result);
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
        if (!perception_.ConvertStaticObstacles(
                actors,
                egoActorId,
                referencePoints,
                &output.static_perception_result))
        {
            *result = output;
            return false;
        }
        output.static_perception_success = true;

        if (!perception_.ConvertDynamicObstacles(
                actors,
                egoActorId,
                referencePoints,
                &output.dynamic_perception_result))
        {
            *result = output;
            return false;
        }
        output.dynamic_perception_success = true;
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
                                   output.static_perception_result.staticobstacles,
                                   &output.dp_result))
        {
            *result = output;
            return false;
        }
        output.dp_success = output.dp_result.dpsuccess;

        // Step 5: DrivableArea — expand coarse DP s/l path into boundaries
        if (!BuildDrivableArea(output.dp_result.path,
                               output.static_perception_result.staticobstacles,
                               &output.drivable_area))
        {
            *result = output;
            return false;
        }
        output.drivable_area_success = true;

        // Step 6: QuadraticProgramming — smooth DP path inside drivable area
        localfrenetpath_.clear();
        if (!RunQuadraticProgramming(output.frenet_start_result,
                                     output.dp_result.path,
                                     output.drivable_area,
                                     output.static_perception_result.staticobstacles,
                                     referencePoints,
                                     &output.qp_result))
        {
            *result = output;
            return false;
        }
        output.qp_success = output.qp_result.qpsuccess;
        output.localfrenetpath = localfrenetpath_;

        if (!QpPathResultToLocalReferenceLinePath(output.qp_result,
                                                  &output.speed_reference_line))
        {
            *result = output;
            return false;
        }

        if (!RunDynamicSpeedPlanning(output.planning_start_result,
                                     output.speed_reference_line,
                                     output.dynamic_perception_result,
                                     &output.speed_result))
        {
            *result = output;
            return false;
        }
        output.speed_success = output.speed_result.dpsuccess;

        if (!BuildTrajectory(output.speed_reference_line,
                             output.speed_result,
                             output.planning_start_result.start_point.time,
                             &output.trajectory))
        {
            *result = output;
            return false;
        }
        output.trajectory_success = true;

        *result = output;
        return output.dp_success &&
               output.drivable_area_success &&
               output.qp_success &&
               output.speed_success &&
               output.trajectory_success;
    }

} // namespace rsim_driver
