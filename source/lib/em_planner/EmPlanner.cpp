#include "EmPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace rsim_driver
{

    namespace
    {

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kEpsilon = 1e-9;

        EmPlannerConfig SynchronizeStaticCollisionConfig(
            EmPlannerConfig config)
        {
            config.drivable_area_config.approach_longitudinal_buffer =
                0.0;
            config.drivable_area_config.departure_longitudinal_buffer =
                config.dp_config.collision.risk_distance -
                config.dp_config.collision.collision_distance;
            config.drivable_area_config.obstacle_transition_length =
                2.0 * config.dp_config.collision.risk_distance;
            config.drivable_area_config.collision_clearance =
                config.dp_config.collision.collision_distance;
            return config;
        }

        double NormalizeAngle(double angle)
        {
            while (angle > kPi)
                angle -= 2.0 * kPi;
            while (angle < -kPi)
                angle += 2.0 * kPi;
            return angle;
        }
        // 后处理准备函数，插值参考线点
        bool InterpolateReferenceLinePoint(
            const std::vector<SpeedReferenceLinePoint> &referenceLine,
            double s,
            SpeedReferenceLinePoint *interpoint)
        {
            if (interpoint == nullptr)
                return false;

            for (std::size_t i = 1; i < referenceLine.size(); ++i)
            {
                const SpeedReferenceLinePoint &previous = referenceLine[i - 1];
                const SpeedReferenceLinePoint &next = referenceLine[i];
                if (s > next.s)
                    continue;

                const double ds = next.s - previous.s;
                const double ratio =
                    std::fabs(ds) > kEpsilon
                        ? std::clamp((s - previous.s) / ds, 0.0, 1.0)
                        : 0.0;

                SpeedReferenceLinePoint point = previous;
                point.x = previous.x + (next.x - previous.x) * ratio;
                point.y = previous.y + (next.y - previous.y) * ratio;
                point.k = previous.k + (next.k - previous.k) * ratio;
                point.hdg = NormalizeAngle(
                    previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
                point.s = s;
                *interpoint = std::move(point);
                return true;
            }

            return false;
        }

        bool IsValidTrajectoryPoint(const PlanningTrajectoryPoint &point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) &&
                   std::isfinite(point.heading) &&
                   std::isfinite(point.curvature) &&
                   std::isfinite(point.speed) && point.speed >= 0.0 &&
                   std::isfinite(point.accel) && std::isfinite(point.time);
        }

    } // namespace

    EmPlanner::EmPlanner(const EmPlannerConfig &config)
        : EMconfig_(SynchronizeStaticCollisionConfig(config)),
          perception_(EMconfig_.perception_config),
          planning_start_(EMconfig_.planning_start_config),
          dp_planner_(EMconfig_.dp_config),
          increase_points_(EMconfig_.dp_increase_points_config),
          drivable_area_builder_(EMconfig_.drivable_area_config),
          qp_path_optimizer_(EMconfig_.qp_config),
          qp_increase_points_(EMconfig_.qp_increase_points_config),
          cutinandout_builder_(EMconfig_.cutinandout_config),
          speed_planner_(EMconfig_.speed_dp_config),
          st_drivable_area_builder_(EMconfig_.speed_drivable_area_config),
          speed_qp_optimizer_(EMconfig_.speed_qp_config),
          speed_qp_increase_points_(EMconfig_.speed_qp_increase_points_config)
    {
    }

    const EmPlannerConfig &EmPlanner::config() const
    {
        return EMconfig_;
    }

    void EmPlanner::SetConfig(const EmPlannerConfig &config)
    {
        EMconfig_ = SynchronizeStaticCollisionConfig(config);
        perception_.SetConfig(EMconfig_.perception_config);
        planning_start_.SetConfig(EMconfig_.planning_start_config);
        dp_planner_.SetConfig(EMconfig_.dp_config);
        increase_points_.SetConfig(EMconfig_.dp_increase_points_config);
        drivable_area_builder_.SetConfig(EMconfig_.drivable_area_config);
        qp_path_optimizer_.SetConfig(EMconfig_.qp_config);
        qp_increase_points_.SetConfig(EMconfig_.qp_increase_points_config);
        cutinandout_builder_.SetConfig(EMconfig_.cutinandout_config);
        speed_planner_.SetConfig(EMconfig_.speed_dp_config);
        st_drivable_area_builder_.SetConfig(
            EMconfig_.speed_drivable_area_config);
        speed_qp_optimizer_.SetConfig(EMconfig_.speed_qp_config);
        speed_qp_increase_points_.SetConfig(
            EMconfig_.speed_qp_increase_points_config);
    }

    bool EmPlanner::SetEgoDimensions(double length, double width)
    {
        if (!std::isfinite(length) || length <= 0.0 ||
            !std::isfinite(width) || width <= 0.0)
        {
            return false;
        }

        if (EMconfig_.dp_config.collision.ego_length == length &&
            EMconfig_.dp_config.collision.ego_width == width &&
            EMconfig_.drivable_area_config.ego_width == width &&
            EMconfig_.qp_config.ego_length == length &&
            EMconfig_.qp_config.ego_width == width &&
            EMconfig_.speed_qp_config.ego_length == length)
        {
            return true;
        }

        EMconfig_.dp_config.collision.ego_length = length;
        EMconfig_.dp_config.collision.ego_width = width;
        EMconfig_.drivable_area_config.ego_width = width;
        EMconfig_.qp_config.ego_length = length;
        EMconfig_.qp_config.ego_width = width;
        EMconfig_.speed_qp_config.ego_length = length;

        dp_planner_.SetConfig(EMconfig_.dp_config);
        drivable_area_builder_.SetConfig(EMconfig_.drivable_area_config);
        qp_path_optimizer_.SetConfig(EMconfig_.qp_config);
        speed_qp_optimizer_.SetConfig(EMconfig_.speed_qp_config);
        return true;
    }

    const PlanningStart &EmPlanner::get_planning_start() const
    {
        return planning_start_;
    }

    const FrenetObstaclePerception &EmPlanner::get_perception() const
    {
        return perception_;
    }

    bool EmPlanner::EMPlanSpeedDetailed(
        const std::vector<rsim_plugin::ActorState> &actors,
        int32_t egoActorId,
        EmPlannerResult *result) const
    {
        if (result == nullptr)
            return false;

        EmPlannerResult output = *result;

        if (!SpeedReferencePathGenerator(output.localcartesianpath,
                                         &output.speed_reference_line))
        {
            *result = output;
            std::cout << "【ST-SpeedReferencePathGenerator】:SL笛卡尔参考线转speed_reference_line失败\n";
            PluginLogEcho("【ST-SpeedReferencePathGenerator】:SL笛卡尔参考线转speed_reference_line失败\n");
            return false;
        }

        if (!perception_.ConvertDynamicObstacles(
                actors,
                egoActorId,
                output.speed_reference_line,
                &output.dynamic_perception_result))
        {
            *result = output;
            std::cout << "【ST-DynamicObstacleConverter】:动态障碍物转换失败\n";
            PluginLogEcho("【ST-DynamicObstacleConverter】:动态障碍物转换失败\n");
            return false;
        }
        output.dynamic_perception_success = true;
        output.perception_success = true;

        // Compute cut-in-and-out boundaries and virtual obstacle seeds.
        // Seeds are written directly into virtual_obstacle_seeds_ (additive).
        std::vector<CutInAndOutInfo> STBoundaryInfos;
        if (!cutinandout_builder_.Compute(
                output.speed_reference_line,
                output.dynamic_perception_result,
                output.planning_start_result.start_point.startpointbasis.speed,
                &STBoundaryInfos))
        {
            std::cout << "【ST-CutInAndOutBuilder】:计算cut-in-and-out边界失败\n";
            PluginLogEcho("【ST-CutInAndOutBuilder】:计算cut-in-and-out边界失败\n");
            *result = output;
            return false;
        }
        //output.virtual_obstacle_seeds = virtual_obstacle_seeds_;

        DynamicPlanSpeedPoint speedStart =
            GetDynamicSpeedPlanStartPoint(output.planning_start_result);
        const double speedPathEnd = output.speed_reference_line.back().s;
        if (!speed_planner_.Plan(speedStart,
                                 STBoundaryInfos,
                                 &output.speed_dp_result,
                                 speedPathEnd,
                                 output.stop_at_reference_end))
        {
            const char *dpMsg =
                output.speed_dp_result.Flag == DynamicPlanSpeedFallback::Stop
                    ? "【ST-DP】：密集障碍物\n"
                    : "【ST-DP】：Other\n";
            std::fprintf(stderr, "%s", dpMsg);
            PluginLogEcho("%s", dpMsg);
            *result = output;
            return false;
        }
        output.speed_dp_success =
            output.speed_dp_result.Flag == DynamicPlanSpeedFallback::Success;

        if (!st_drivable_area_builder_.Build(STBoundaryInfos,
                                             &output.speed_dp_result,
                                             &output.drivable_area_st))
        {
            const char *stMsg =
                output.drivable_area_st.Flag == StDrivableAreaFallback::Stop
                    ? "【ST-DrivableArea】：ST可行驶区域过窄\n"
                    : "【ST-DrivableArea】：Other\n";
            std::fprintf(stderr, "%s", stMsg);
            PluginLogEcho("%s", stMsg);
            *result = output;
            return false;
        }
        output.st_drivable_area_success = output.drivable_area_st.Flag == StDrivableAreaFallback::Success;

        if (!speed_qp_optimizer_.Optimize(speedStart,
                                          output.drivable_area_st,
                                          &output.speed_qp_result,
                                          output.stop_at_reference_end))
        {
            const char *qpMsg =
                output.speed_qp_result.Flag == QpSpeedOptimizerFallback::Stop
                    ? "【ST-QP】：QP优化：求解失败\n"
                    : "【ST-QP】：Other\n";
            std::fprintf(stderr, "%s", qpMsg);
            PluginLogEcho("%s", qpMsg);
            *result = output;
            return false;
        }
        output.speed_qp_success = output.speed_qp_result.Flag == QpSpeedOptimizerFallback::Success;

        if (!speed_qp_increase_points_.increasepoints(
                output.speed_qp_result,
                &output.speed_increasepoints_line))
        {
            *result = output;
            return false;
        }
        output.speed_qp_increase_points_success = true;

        *result = std::move(output);
        return true;
    }

    bool EmPlanner::BuildTrajectory(
        const std::vector<SpeedReferenceLinePoint> &speedreferenceline,
        const std::vector<DynamicPlanSpeedPoint> &speedincreaseline,
        const PlanningStartResult &planningStartResult,
        std::vector<PlanningTrajectoryPoint> *result) const
    {
        if (result == nullptr)
            return false;
        result->clear();
        // 输入参数检查
        if (speedreferenceline.empty() || speedincreaseline.empty())
        {
            std::cout << "【PostProcess】：后处理失败:SL参考线或者增密ST线为空\n"
                      << std::endl;
            PluginLogEcho("【PostProcess】：后处理失败:SL参考线或者增密ST线为空\n");
            return false;
        }
        SpeedReferenceLinePoint pathPoint;
        PlanningTrajectoryPoint point;
        for (const auto &speedPoint : speedincreaseline)
        {
            pathPoint = {};
            point = {};
            if (!InterpolateReferenceLinePoint(speedreferenceline, speedPoint.s, &pathPoint))
            {
                result->clear();
                std::cout << "【PostProcess】：后处理失败:插值SL参考线点失败\n"
                          << std::endl;
                PluginLogEcho("【PostProcess】：后处理失败:插值SL参考线点失败\n");
                return false;
            }

            point.x = pathPoint.x;
            point.y = pathPoint.y;
            point.heading = pathPoint.hdg;
            point.curvature = pathPoint.k;
            point.speed = speedPoint.v;
            point.accel = speedPoint.a;
            point.time = planningStartResult.start_point.startpointbasis.time + speedPoint.t;
            if (!IsValidTrajectoryPoint(point))
            {
                result->clear();
                std::cout << "【PostProcess】：后处理失败:轨迹点参数不合理\n"
                          << std::endl;
                PluginLogEcho("【PostProcess】：后处理失败:轨迹点参数不合理\n");
                return false;
            }
            result->push_back(point);
        }

        return true;
    }

    bool EmPlanner::EMPlanPostProcessDetailed(
        EmPlannerResult &sltoutput) const
    {
        EmPlannerResult output = sltoutput;

        const bool forwardTrajectory = BuildTrajectory(output.speed_reference_line,
                                                       output.speed_increasepoints_line,
                                                       output.planning_start_result,
                                                       &output.trajectory);
        if (!forwardTrajectory)
        {
            sltoutput = std::move(output);
            return false;
        }

        if (output.planning_start_result.start_point.source ==
            PlanningStartSource::PreviousTrajectory)
        {
            output.trajectory.insert(output.trajectory.begin(),
                                     output.planning_start_result.stitching_trajectory.begin(),
                                     output.planning_start_result.stitching_trajectory.end());
        }
        for (std::size_t i = 0; i < output.trajectory.size(); ++i)
        {
            if ((i > 0 &&
                 output.trajectory[i].time - output.trajectory[i - 1].time <= kEpsilon))
            {
                output.trajectory.clear();
                std::cout << "【PostProcess】：后处理失败:轨迹点时间间隔过小或时序混乱\n"
                          << std::endl;
                PluginLogEcho("【PostProcess】：后处理失败:轨迹点时间间隔过小或时序混乱\n");
                sltoutput = std::move(output);
                return false;
            }
        }
        output.trajectory_success = true;
        sltoutput = std::move(output);
        return true;
    }

} // namespace rsim_driver
