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

        double NormalizeAngle(double angle)
        {
            while (angle > kPi)
                angle -= 2.0 * kPi;
            while (angle < -kPi)
                angle += 2.0 * kPi;
            return angle;
        }
        // 后处理准备函数，插值参考线点
        localreferencelinepoint InterpolateReferenceLinePoint(
            const localreferencelinepath &referenceLine,
            double s)
        {
            if (referenceLine.empty())
                return {};
            if (referenceLine.size() == 1 || s <= referenceLine.front().s)
                return referenceLine.front();
            if (s >= referenceLine.back().s)
                return referenceLine.back();

            for (std::size_t i = 1; i < referenceLine.size(); ++i)
            {
                const localreferencelinepoint &previous = referenceLine[i - 1];
                const localreferencelinepoint &next = referenceLine[i];
                if (s > next.s)
                    continue;

                const double ds = next.s - previous.s;
                const double ratio =
                    std::fabs(ds) > kEpsilon
                        ? std::clamp((s - previous.s) / ds, 0.0, 1.0)
                        : 0.0;

                localreferencelinepoint point = previous;
                point.x = previous.x + (next.x - previous.x) * ratio;
                point.y = previous.y + (next.y - previous.y) * ratio;
                point.k = previous.k + (next.k - previous.k) * ratio;
                point.hdg = NormalizeAngle(
                    previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
                point.s = s;
                // point.dk = previous.dk + (next.dk - previous.dk) * ratio;
                return point;
            }

            return referenceLine.back();
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
        : EMconfig_(config),
          perception_(config.perception_config),
          planning_start_(config.planning_start_config),
          dp_planner_(config.dp_config),
          increase_points_(config.dp_increase_points_config),
          drivable_area_builder_(config.drivable_area_config),
          qp_path_optimizer_(config.qp_config),
          qp_increase_points_(config.qp_increase_points_config),
          speed_planner_(config.speed_dp_config),
          speed_qp_optimizer_(config.speed_qp_config)
    {
    }

    const EmPlannerConfig &EmPlanner::config() const
    {
        return EMconfig_;
    }

    void EmPlanner::SetConfig(const EmPlannerConfig &config)
    {
        EMconfig_ = config;
        perception_.SetConfig(config.perception_config);
        planning_start_.SetConfig(config.planning_start_config);
        dp_planner_.SetConfig(config.dp_config);
        increase_points_.SetConfig(config.dp_increase_points_config);
        drivable_area_builder_.SetConfig(config.drivable_area_config);
        qp_path_optimizer_.SetConfig(config.qp_config);
        qp_increase_points_.SetConfig(config.qp_increase_points_config);
        speed_planner_.SetConfig(config.speed_dp_config);
        speed_qp_optimizer_.SetConfig(config.speed_qp_config);
    }

    const PlanningStart &EmPlanner::get_planning_start() const
    {
        return planning_start_;
    }

    const FrenetObstaclePerception &EmPlanner::get_perception() const
    {
        return perception_;
    }

    const std::vector<CartesianPathPoint> &EmPlanner::get_local_cartesian_path() const
    {
        return localcartesianpath_;
    }

    const localreferencelinepath &EmPlanner::get_speed_reference_line() const
    {
        return speed_reference_line_;
    }


    bool EmPlanner::BuildTrajectory(
        const localreferencelinepath &referenceLine,
        const std::vector<DynamicPlanSpeedPoint> &speedPoints,
        const PlanningStartResult &planningStartResult,
        std::vector<PlanningTrajectoryPoint> *result) const
    {
        if (result == nullptr)
            return false;

        result->clear();
        for (const auto &speedPoint : speedPoints)
        {
            const localreferencelinepoint pathPoint =
                InterpolateReferenceLinePoint(referenceLine, speedPoint.s);

            PlanningTrajectoryPoint point;
            point.x = pathPoint.x;
            point.y = pathPoint.y;
            point.heading = pathPoint.hdg;
            point.curvature = pathPoint.k;
            point.speed = speedPoint.v;
            point.accel = speedPoint.a;
            point.time = planningStartResult.start_point.time + speedPoint.t;
            if (!IsValidTrajectoryPoint(point))
            {
                result->clear();
                return false;
            }
            result->push_back(point);
        }

        return true;
    }

    bool EmPlanner::EMPlanSpeedDetailed(
        const std::vector<rsim_plugin::ActorState> &actors,
        int32_t egoActorId,
        const PlanningStartResult &planningStartResult,
        EmPlannerResult *result) const
    {
        if (result == nullptr)
            return false;

        EmPlannerResult output = *result;
        output.speed_reference_line.clear();

        if (!LocalCartesianPathToReferenceLinePath(localcartesianpath_,
                                                   &output.speed_reference_line))
        {
            *result = output;
            return false;
        }
        speed_reference_line_.clear();
        speed_reference_line_ = output.speed_reference_line;

        if (!perception_.ConvertDynamicObstacles(
                actors,
                egoActorId,
                output.speed_reference_line,
                &output.dynamic_perception_result))
        {
            *result = output;
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
                planningStartResult.start_point.speed,
                EMconfig_.planning_start_config.planning_period,
                EMconfig_.speed_dp_config.time_step *
                    static_cast<double>(EMconfig_.speed_dp_config.time_step_count),
                &STBoundaryInfos,
                &virtual_obstacle_seeds_))
        {
            *result = output;
            return false;
        }
        output.virtual_obstacle_seeds = virtual_obstacle_seeds_;

        DynamicPlanSpeedPoint speedStart =
            GetDynamicSpeedPlanStartPoint(planningStartResult);
        if (!speed_planner_.Plan(speedStart, STBoundaryInfos, &output.speed_dp_result))
        {
            output.speed_dp_result.Flag == DpPlannerFallback::Stop ? cout << "ST-DP规划：密集障碍物" << endl : cout << "Other" << endl;
            *result = output;
            return false;
        }
        output.speed_dp_success = output.speed_dp_result.Flag == DpPlannerFallback::Success;

        if (!st_drivable_area_builder_.Build(STBoundaryInfos,
                                             output.speed_dp_result,
                                             &output.drivable_area_st))
        {
            output.drivable_area_st.Flag == DpPlannerFallback::Stop ? cout << "ST可行驶区域过窄" << endl : cout << "Other" << endl;
            *result = output;
            return false;
        }
        output.st_drivable_area_success = output.drivable_area_st.Flag == StDrivableAreaFallback::Success;

        if (!speed_qp_optimizer_.Optimize(speedStart,
                                          output.drivable_area_st,
                                          &output.speed_qp_result))
        {
            output.speed_qp_result.Flag == QpSpeedOptimizerFallback::Stop ? cout << "ST-QP优化：求解失败" << endl : cout << "Other" << endl;
            *result = output;
            return false;
        }
        output.speed_qp_success = output.speed_qp_result.Flag == QpSpeedOptimizerFallback::Success;

        if (!speed_qp_increase_points_.increasepoints(
                output.speed_qp_result,
                &output.increasepoints_speed_points))
        {
            *result = output;
            return false;
        }
        output.speed_qp_increase_points_success = true;

        *result = std::move(output);
        return true;
    }

    bool EmPlanner::EMPlanPostProcessDetailed(
        const PlanningStartResult &planningStartResult,
        const std::vector<DynamicPlanSpeedPoint> &speedPoints,
        std::vector<PlanningTrajectoryPoint> *result) const
    {
        if (result == nullptr)
            return false;

        const bool forwardTrajectory = BuildTrajectory(speed_reference_line_,
                                                       speedPoints,
                                                       planningStartResult,
                                                       result);
        if (!forwardTrajectory)
            return false;

        if (planningStartResult.start_point.source ==
            PlanningStartSource::PreviousTrajectory)
        {
            result->insert(result->begin(),
                           planningStartResult.stitching_trajectory.begin(),
                           planningStartResult.stitching_trajectory.end());
        }
        for (std::size_t i = 0; i < result->size(); ++i)
        {
            if (!IsValidTrajectoryPoint((*result)[i]) ||
                (i > 0 &&
                 (*result)[i].time - (*result)[i - 1].time <= kEpsilon))
            {
                result->clear();
                return false;
            }
        }
        return true;
    }

} // namespace rsim_driver
