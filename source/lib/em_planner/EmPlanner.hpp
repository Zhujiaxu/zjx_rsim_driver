#pragma once

#include "perception/FrenetObstaclePerception.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "local_path_planning/drivable_area/DrivableArea.hpp"
#include "local_path_planning/dpplanner/DpPlanner.hpp"
#include "local_path_planning/qpplanner/QpPathOptimizer.hpp"
#include "local_speed_planning/dpspeedplanner/DynamicSpeedPlanner.hpp"
#include "local_speed_planning/computecutinandout/computecutinandout.hpp"
#include "local_path_planning/dp_increase_points/dpincreasepoints.hpp"
#include "local_path_planning/qp_increase_points/qpincreasepoints.hpp"
#include "local_speed_planning/speed_drivable_area/StDrivableArea.hpp"
#include "local_speed_planning/speed_quadratic_programming/SpeedQpOptimizer.hpp"
#include "local_speed_planning/speed_qp_increase_points/SpeedQpIncreasePoints.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace rsim_driver
{

    struct EmPlannerConfig
    {
        FrenetObstaclePerceptionConfig perception_config;
        PlanningStartConfig planning_start_config;
        DpPlannerConfig dp_config;
        DPIncreasePointsConfig dp_increase_points_config;
        DrivableAreaConfig drivable_area_config;
        QpPathOptimizerConfig qp_config;
        QpIncreasePointsConfig qp_increase_points_config;
        DynamicPlanSpeedConfig speed_dp_config;
        StDrivableAreaConfig speed_drivable_area_config;
        QpSpeedOptimizerConfig speed_qp_config;
        QpSpeedIncreasePointsConfig speed_qp_increase_points_config;
    };

    struct EmPlannerResult
    {

        bool perception_success = false;
        bool static_perception_success = false;
        bool dynamic_perception_success = false;
        bool virtual_perception_success = false;
        bool planning_start_success = false;
        bool frenet_start_success = false;
        bool dp_success = false;
        bool dp_increase_points_success = false;
        bool drivable_area_success = false;
        bool qp_success = false;
        bool qp_increase_points_success = false;
        bool qp_increase_points_to_frenet=false;
        bool speed_dp_success = false;
        bool st_drivable_area_success = false;
        bool speed_qp_success = false;
        bool speed_qp_increase_points_success = false;
        bool trajectory_success = false;
        PlanningStartResult planning_start_result;
        StartPointFrenetState frenet_start_result;
        StaticFrenetObstaclePerceptionResult static_perception_result;
        DynamicFrenetObstaclePerceptionResult dynamic_perception_result;
        VirtualFrenetObstaclePerceptionResult virtual_perception_result;
        std::vector<VirtualObstacleSeed> virtual_obstacle_seeds;
        DpPlannerResult dp_result;
        DpIncreasePointsResult dp_increase_points_result;
        DrivableAreaResult drivable_area_result;
        QpPathResult qp_result;
        QpIncreasePointsResult qp_increase_points_result;
        std::vector<CartesianPathPoint> localcartesianpath;
        std::vector<SpeedReferenceLinePoint> speed_reference_line;
        DynamicPlanSpeedResult speed_dp_result;
        StDrivableAreaResult drivable_area_st;
        QpSpeedOptimizerResult speed_qp_result;
        std::vector<DynamicPlanSpeedPoint> speed_increasepoints_line;
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
            EmPlannerResult *result) const;

        bool EMPlanPostProcessDetailed(
             EmPlannerResult &sltoutput) const;

        const PlanningStart &get_planning_start() const;
        const FrenetObstaclePerception &get_perception() const;
    private:
        bool BuildTrajectory(
            const std::vector<SpeedReferenceLinePoint> &referenceLine,
            const std::vector<DynamicPlanSpeedPoint> &newqppointspath,
            const PlanningStartResult &planningStartResult,
            std::vector<PlanningTrajectoryPoint> *result) const;

        EmPlannerConfig EMconfig_;
        FrenetObstaclePerception perception_;
        PlanningStart planning_start_;
        DpPlanner dp_planner_;
        DPIncreasePoints increase_points_;
        DrivableAreaBuilder drivable_area_builder_;
        QpPathOptimizer qp_path_optimizer_;
        QpIncreasePoints qp_increase_points_;
        ComputeCutInAndOut cutinandout_builder_;
        DynamicPlanSpeedPlanner speed_planner_;
        StDrivableAreaBuilder st_drivable_area_builder_;
        SpeedQpOptimizer speed_qp_optimizer_;
        QpSpeedIncreasePoints speed_qp_increase_points_;
        mutable std::vector<VirtualObstacleSeed> virtual_obstacle_seeds_;
    };

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

        if (!perception_.ConvertVirtualObstacles(
                actors,
                egoActorId,
                referencePoints,
                virtual_obstacle_seeds_,
                &output.virtual_perception_result))
        {
            *result = output;
            return false;
        }
        output.virtual_perception_success = true;
        output.virtual_obstacle_seeds = virtual_obstacle_seeds_;

        std::vector<StaticFrenetObstacle> pathObstacles =
            output.static_perception_result.staticobstacles;
        pathObstacles.insert(pathObstacles.end(),
                             output.virtual_perception_result.virtualstaticobstacles.begin(),
                             output.virtual_perception_result.virtualstaticobstacles.end());

        // Step 2: PlanningStart — compute start point in Cartesian
        if (!planning_start_.Compute(ego,
                                     currentTime,
                                     previousTrajectory,
                                     &output.planning_start_result))
        {
            *result = output;
            return false;
        }
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
        if (!dp_planner_.Plan(output.frenet_start_result,
                              pathObstacles,
                              &output.dp_result))
        {
            output.dp_result.Flag == DpPlannerFallback::Stop 
            ? std::cout << "【SL-DP】:密集障碍物||规划起点已碰撞，紧急停车/n" << std::endl
            : std::cout << "【SL-DP】:其他错误" << std::endl;

            *result = output;
            return false;
        }
        output.dp_success = output.dp_result.Flag == DpPlannerFallback::Success;

        if (!increase_points_.increasepoints(output.dp_result, &output.dp_increase_points_result))
        {
            *result = output;
            return false;
        }
        output.dp_increase_points_success = true;
        // Step 5: DrivableArea — expand coarse DP s/l path into boundaries
        if (!drivable_area_builder_.Build(output.dp_increase_points_result.path,
                                          pathObstacles,
                                          &output.drivable_area_result))
        {
            output.drivable_area_result.Flag == DrivableAreaFallback::Stop ? std::cout << "【SL-DriArea】:可行使区域过窄" << std::endl  : std::cout << "【SL-DriArea】:可行驶区域其他错误" << std::endl ;
            *result = output;
            return false;
        }
        output.drivable_area_success = output.drivable_area_result.Flag == DrivableAreaFallback::Success;

        // Step 6: QuadraticProgramming — smooth DP path inside drivable area

        if (!qp_path_optimizer_.Optimize(
                output.frenet_start_result,
                output.drivable_area_result,
                &output.qp_result))
        {
            output.qp_result.Flag == QpPathOptimizerFallback::SolveFailStop ? std::cout << "【SL-QP】:QP求解失败" << std::endl : std::cout << "【SL-QP】:QP Path其他错误" << std::endl ;
            *result = output;
            return false;
        }

        output.qp_success = output.qp_result.Flag == QpPathOptimizerFallback::Success;


        if (!qp_increase_points_.increasepoints(output.qp_result,
                                                &output.qp_increase_points_result))
        {
            *result = output;
            return false;
        }
        output.qp_increase_points_success = true;


        std::vector<CartesianPathPoint> CartesianPath;
        if (!FrenetPathToCartesian(referencePoints,
                                   output.qp_increase_points_result.localfrenetpath,
                                   &CartesianPath))
        {
            *result = output;
            return false;
        }
        output.qp_increase_points_to_frenet = true;
        output.localcartesianpath = std::move(CartesianPath);

        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
