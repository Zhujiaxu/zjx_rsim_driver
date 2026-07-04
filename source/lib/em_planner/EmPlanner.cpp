#include "EmPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

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
                point.theta = NormalizeAngle(
                    previous.theta + NormalizeAngle(next.theta - previous.theta) * ratio);
                point.s = s;
                // point.dk = previous.dk + (next.dk - previous.dk) * ratio;
                return point;
            }

            return referenceLine.back();
        }

    } // namespace

    EmPlanner::EmPlanner(const EmPlannerConfig &config)
        : EMconfig_(config),
          perception_(config.perception_config),
          planning_start_(config.planning_start_config),
          dp_planner_(config.dp_config),
          drivable_area_builder_(config.drivable_area_config),
          qp_path_optimizer_(config.qp_config),
          speed_planner_(config.speed_config)
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
        drivable_area_builder_.SetConfig(config.drivable_area_config);
        qp_path_optimizer_.SetConfig(config.qp_config);
        speed_planner_.SetConfig(config.speed_config);
    }

    const FrenetObstaclePerception &EmPlanner::get_perception() const
    {
        return perception_;
    }

    const PlanningStart &EmPlanner::get_planning_start() const
    {
        return planning_start_;
    }

    bool EmPlanner::RunDynamicProgramming(
        const CartesianFrenetState &start,
        const std::vector<StaticFrenetObstacle> &obstacles,
        DpPlannerResult *result) const
    {
        return dp_planner_.Plan(start, obstacles, result);
    }

    bool EmPlanner::BuildDrivableArea(
        const std::vector<DpPathPoint> &coarsePath,
        const std::vector<StaticFrenetObstacle> &obstacles,
        DrivableArea *result) const
    {
        return drivable_area_builder_.Build(coarsePath, obstacles, result);
    }

    bool EmPlanner::RunDynamicSpeedPlanning(
        const PlanningStartResult &start,
        const localreferencelinepath &referenceLine,
        const DynamicFrenetObstaclePerceptionResult &dynamicObstacles,
        DynamicPlanSpeedResult *result) const
    {
        if (referenceLine.empty())
            return false;

        DynamicPlanSpeedPoint speedStart =
            GetDynamicSpeedPlanStartPoint(start);
        // speedStart.s = referenceLine.front().s;
        return speed_planner_.Plan(speedStart,
                                   referenceLine,
                                   dynamicObstacles,
                                   result);
    }

    bool EmPlanner::BuildTrajectory(
        const localreferencelinepath &referenceLine,
        const DynamicPlanSpeedResult &speedResult,
        const PlanningStartResult &planningStartResult,
        std::vector<PlanningTrajectoryPoint> *result) const
    {
        if (result == nullptr)
            return false;

        result->clear();
        if (referenceLine.empty() || speedResult.stpoints.empty())
            return false;

        result->reserve(speedResult.stpoints.size());
        for (const DynamicPlanSpeedPoint &speedPoint : speedResult.stpoints)
        {
            const localreferencelinepoint pathPoint =
                InterpolateReferenceLinePoint(referenceLine, speedPoint.s);

            PlanningTrajectoryPoint point;
            point.x = pathPoint.x;
            point.y = pathPoint.y;
            point.heading = pathPoint.theta;
            point.curvature = pathPoint.k;
            point.speed = speedPoint.v;
            point.accel = speedPoint.a;
            point.time = planningStartResult.start_point.time + speedPoint.t;
            if (planningStartResult.start_point.source == PlanningStartSource::PreviousTrajectory)
            {
                for (const PlanningTrajectoryPoint &point : planningStartResult.stitching_trajectory)
                {
                    result->push_back(point);
                }
            }
            result->push_back(point);
        }

        return true;
    }

    bool EmPlanner::EMPlanSpeedDetailed(
        const PlanningStartResult &planningStartResult,
        const DynamicFrenetObstaclePerceptionResult &dynamicObstacles,
        const QpPathResult &qpPathResult,
        EmPlannerResult *result) const
    {
        if (result == nullptr)
            return false;

        EmPlannerResult output = *result;
        output.speed_reference_line.clear();
        output.speed_result = {};
        output.speed_success = false;

        if (!QpPathResultToLocalReferenceLinePath(qpPathResult,
                                                  &output.speed_reference_line))
        {
            *result = output;
            return false;
        }

        if (!RunDynamicSpeedPlanning(planningStartResult,
                                     output.speed_reference_line,
                                     dynamicObstacles,
                                     &output.speed_result))
        {
            *result = output;
            return false;
        }
        output.speed_success = output.speed_result.dpsuccess;

        *result = output;
        return output.speed_success;
    }

    bool EmPlanner::EMPlanPostProcessDetailed(
        const PlanningStartResult &planningStartResult,
        const QpPathResult &qpPathResult,
        const DynamicPlanSpeedResult &speedResult,
        std::vector<PlanningTrajectoryPoint> *result) const
    {
        if (result == nullptr)
            return false;

        localreferencelinepath referenceLine;
        if (!QpPathResultToLocalReferenceLinePath(qpPathResult, &referenceLine))
        {
            result->clear();
            return false;
        }

        return BuildTrajectory(referenceLine,
                               speedResult,
                               planningStartResult,
                               result);
    }

} // namespace rsim_driver
