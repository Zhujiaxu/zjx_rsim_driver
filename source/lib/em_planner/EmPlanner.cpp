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
                point.hdg = NormalizeAngle(
                    previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
                point.s = s;
                // point.dk = previous.dk + (next.dk - previous.dk) * ratio;
                return point;
            }

            return referenceLine.back();
        }

        DynamicPlanSpeedConfig BuildSpeedConfig(const EmPlannerConfig &config)
        {
            DynamicPlanSpeedConfig speedConfig = config.speed_config;
            speedConfig.planning_period =
                std::max(0.0, config.planning_start_config.planningPeriod);
            return speedConfig;
        }

    } // namespace

    EmPlanner::EmPlanner(const EmPlannerConfig &config)
        : EMconfig_(config),
          perception_(config.perception_config),
          planning_start_(config.planning_start_config),
          dp_planner_(config.dp_config),
          drivable_area_builder_(config.drivable_area_config),
          qp_path_optimizer_(config.qp_config),
          speed_planner_(BuildSpeedConfig(config))
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
        speed_planner_.SetConfig(BuildSpeedConfig(config));
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
            point.heading = pathPoint.hdg;
            point.curvature = pathPoint.k;
            point.speed = speedPoint.v;
            point.accel = speedPoint.a;
            point.time = planningStartResult.start_point.time + speedPoint.t;
            result->push_back(point);
        }

        return true;
    }

    void EmPlanner::MergeVirtualObstacleSeeds(
        const std::vector<VirtualObstacleSeed> &seeds) const
    {
        for (const VirtualObstacleSeed &seed : seeds)
        {
            const auto sameSeed =
                [&seed](const VirtualObstacleSeed &existing)
            {
                return existing.source_actor_id == seed.source_actor_id &&
                       existing.type == seed.type;
            };
            auto existing = std::find_if(virtual_obstacle_seeds_.begin(),
                                         virtual_obstacle_seeds_.end(),
                                         sameSeed);
            if (existing != virtual_obstacle_seeds_.end())
            {
                *existing = seed;
            }
            else
            {
                virtual_obstacle_seeds_.push_back(seed);
            }
        }
    }

    void EmPlanner::PruneVirtualObstacleSeeds(
        const std::vector<StaticFrenetObstacle> &resolvedObstacles,
        double planningStartS) const
    {
        virtual_obstacle_seeds_.erase(
            std::remove_if(
                virtual_obstacle_seeds_.begin(),
                virtual_obstacle_seeds_.end(),
                [&](const VirtualObstacleSeed &seed)
                {
                    const auto resolved =
                        std::find_if(resolvedObstacles.begin(),
                                     resolvedObstacles.end(),
                                     [&](const StaticFrenetObstacle &obstacle)
                                     {
                                         return obstacle.id ==
                                                seed.source_actor_id;
                                     });
                    if (resolved == resolvedObstacles.end())
                        return true;

                    const double obstacleTailS =
                        resolved->s + 0.5 * std::max(0.0, resolved->length);
                    return obstacleTailS < planningStartS;
                }),
            virtual_obstacle_seeds_.end());
    }

    bool EmPlanner::EMPlanSpeedDetailed(
        const std::vector<rsim_plugin::ActorState> &actors,
        int32_t egoActorId,
        const PlanningStartResult &planningStartResult,
        // const DynamicFrenetObstaclePerceptionResult &dynamicObstacles,
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

        output.dynamic_perception_success = false;
        output.perception_success = output.static_perception_success;
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

        if (!RunDynamicSpeedPlanning(planningStartResult,
                                     output.speed_reference_line,
                                     // dynamicObstacles,
                                     output.dynamic_perception_result,
                                     &output.speed_result))
        {
            *result = output;
            return false;
        }
        output.speed_success = output.speed_result.dpsuccess;
        output.virtual_obstacle_seeds =
            output.speed_result.virtual_obstacle_seeds;
        MergeVirtualObstacleSeeds(output.speed_result.virtual_obstacle_seeds);
        output.virtual_obstacle_seeds = virtual_obstacle_seeds_;

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

        const bool forwardTrajectory = BuildTrajectory(referenceLine,
                                                       speedResult,
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
        return true;
    }

} // namespace rsim_driver
