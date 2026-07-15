#pragma once

#include "perception/FrenetObstaclePerception.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "local_path_planning/drivable_area/DrivableArea.hpp"
#include "local_path_planning/dynamic_programming/DpPlanner.hpp"
#include "local_path_planning/quadratic_programming/QpPathOptimizer.hpp"
#include "local_speed_planning/dynamic_speed_planning/DynamicSpeedPlanner.hpp"
#include "local_speed_planning/computecutinandout/computecutinandout.hpp"
#include "local_path_planning/dp_increase_points/dpincreasepoints.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rsim_driver
{

    struct EmPlannerConfig
    {
        FrenetObstaclePerceptionConfig perception_config;
        PlanningStartConfig planning_start_config;
        DpPlannerConfig dp_config;
        DPIncreasePointsConfig increase_points_config;
        DrivableAreaConfig drivable_area_config;
        QpPathOptimizerConfig qp_config;
        DynamicPlanSpeedConfig speed_config;
    };

    struct EmPlannerResult
    {

        bool perception_success = false;
        bool static_perception_success = false;
        bool dynamic_perception_success = false;
        bool planning_start_success = false;
        bool frenet_start_success = false;
        bool dp_success = false;
        bool increase_points_success = false;
        bool drivable_area_success = false;
        bool qp_success = false;
        bool speed_success = false;
        bool trajectory_success = false;
        PlanningStartResult planning_start_result;
        CartesianFrenetState frenet_start_result;
        StaticFrenetObstaclePerceptionResult static_perception_result;
        DynamicFrenetObstaclePerceptionResult dynamic_perception_result;
        std::vector<VirtualFrenetObstacle> virtual_static_obstacles;
        std::vector<VirtualObstacleSeed> virtual_obstacle_seeds;
        DpPlannerResult dp_result;
        DrivableArea drivable_area;
        std::vector<DpPathPoint> localfrenetpath;
        QpPathResult qp_result;
        localreferencelinepath speed_reference_line;
        DynamicPlanSpeedResult speed_result;
        std::vector<PlanningTrajectoryPoint> trajectory;
    };

    class EmPlanner
    {
    public:
        explicit EmPlanner(const EmPlannerConfig &config = {});

        const EmPlannerConfig &config() const;
        void SetConfig(const EmPlannerConfig &config);

        template <typename RefPointT>
        bool EMPlanPathDetailed(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const rsim_plugin::ActorState &ego,
            double currentTime,
            const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
            const std::vector<RefPointT> &referencePoints,
            EmPlannerResult *result) const;

        bool EMPlanSpeedDetailed(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const PlanningStartResult &planningStartResult,
            const QpPathResult &qpPathResult,
            EmPlannerResult *result) const;

        bool EMPlanPostProcessDetailed(
            const PlanningStartResult &planningStartResult,
            const QpPathResult &qpPathResult,
            const DynamicPlanSpeedResult &speedResult,
            std::vector<PlanningTrajectoryPoint> *result) const;

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
            const std::vector<CutInAndOutInfo> &STBoundaryInfos,
            DynamicPlanSpeedResult *result) const;
        bool BuildTrajectory(
            const localreferencelinepath &referenceLine,
            const DynamicPlanSpeedResult &speedResult,
            const PlanningStartResult &planningStartResult,
            std::vector<PlanningTrajectoryPoint> *result) const;
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
        DPIncreasePoints increase_points_;
        DrivableAreaBuilder drivable_area_builder_;
        QpPathOptimizer qp_path_optimizer_;
        DynamicPlanSpeedPlanner speed_planner_;
        mutable ComputeCutInAndOut cut_in_and_out_builder_;
        mutable std::vector<DpPathPoint> localfrenetpath_;
        mutable std::vector<VirtualObstacleSeed> virtual_obstacle_seeds_;
    };

    // --- template implementation ---

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
    bool EmPlanner::EMPlanPathDetailed(
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

        if (!perception_.ConvertVirtualObstacles(
                actors,
                egoActorId,
                referencePoints,
                virtual_obstacle_seeds_,
                &output.virtual_static_obstacles))
        {
            *result = output;
            return false;
        }
            output.virtual_obstacle_seeds = virtual_obstacle_seeds_;

        std::vector<StaticFrenetObstacle> pathObstacles =
            output.static_perception_result.staticobstacles;
        pathObstacles.insert(pathObstacles.end(),
                             output.virtual_static_obstacles.begin(),
                             output.virtual_static_obstacles.end());

        // Step 4: Dynamic Programming — plan path
        if (!RunDynamicProgramming(output.frenet_start_result,
                                   pathObstacles,
                                   &output.dp_result))
        {
            *result = output;
            return false;
        }
        output.dp_success = output.dp_result.dpsuccess;

        // Stop fallback: DP blocked, let vehicle wait
        if (output.dp_result.fallback == DpFallback::Stop)
        {
            *result = output;
            return true;
        }

        DpPlannerResult newresult;
        if (!increase_points_.increasepoints(&output.dp_result, &newresult))
        {
            output.increase_points_success = false;
            output.dp_success = false;
            *result = output;
            return false;
        }
        output.increase_points_success = true;
        output.dp_result = newresult;
        // Step 5: DrivableArea — expand coarse DP s/l path into boundaries
        if (!BuildDrivableArea(output.dp_result.path,
                               pathObstacles,
                               &output.drivable_area))
        {
            *result = output;
            return false;
        }
        output.drivable_area_success = true;

        // Step 6: QuadraticProgramming — smooth DP path inside drivable area
        localfrenetpath_.clear();
        const bool qpComputed = RunQuadraticProgramming(
            output.frenet_start_result,
            output.dp_result.path,
            output.drivable_area,
            pathObstacles,
            referencePoints,
            &output.qp_result);
        if (!qpComputed)
        {
            *result = output;
            return false;
        }
        output.qp_success = output.qp_result.qpsuccess;
        output.localfrenetpath = localfrenetpath_;

        *result = output;
        return output.dp_success &&
               output.drivable_area_success &&
               output.qp_success;
    }

} // namespace rsim_driver
