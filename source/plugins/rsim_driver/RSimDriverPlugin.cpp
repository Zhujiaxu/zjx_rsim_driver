/*
 * ============================================================================
 * RSimDriverPlugin — 全局路径获取插件
 * ============================================================================
 *
 * ---- 整体数据流 ----
 *
 *   xosc 场景文件
 *     └── Init → FollowTrajectoryAction (ego 的初始轨迹)
 *            │
 *            ▼
 *   RSimDriverPlugin::Init()
 *     └── 调用 global_path 静态库解析 runtime XOSC
 *            └── Init/Private[@entityRef="ego"]/FollowTrajectoryAction
 *            │
 *            ▼
 *   RSimDriverPlugin::Step() (每帧)

 *     └── EmPlanner 输出 QP local Cartesian 路径 → 控制目标点
 *
 * ---- 全局路径数据结构 ----
 *
 *   global_path 静态库输出两份数据:
 *     1. route_segments_[]          — 路段索引 (road_id, lane_id, s区间)
 *                                    用于 road/lane tracking
 *     2. global_path_world_points_[] — 世界坐标点序列 (x, y)
 *                                    供下游规划模块构建参考线
 *
 * ---- 配置参数 (xosc properties / plugin.yaml) ----
 *
 *   xodrPath     : OpenDRIVE 地图绝对路径 (必需)
 *   routeXoscPath: runtime OpenSCENARIO 路径, 用于读取 ego FollowTrajectory
 *   entityName   : 要读取 FollowTrajectory 的实体名 (默认 "ego")
 *   setSpeed     : 期望巡航速度 m/s (默认 10.0, 预留)
 *   referenceLineCsvPath: 参考线运动调试 CSV 输出路径 (可选)
 *   obstacleCsvPath: 障碍物转化 CSV 输出路径 (可选)
 *   planningStartSlCsvPath: 规划起点 Frenet 调试 CSV 输出路径 (可选)
 *   egoTrajectoryCsvPath: QP local Frenet/Cartesian 规划轨迹 CSV 输出路径 (可选)
 * ============================================================================
 */

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include "EmPlanner.hpp"
#include "FrenetToCartesian.hpp"
#include "GlobalPathGenerator.hpp"
#include "MapHelper.hpp"
#include "ObstacleToCsv.hpp"
#include "ReferenceLineGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using rsim_plugin::ActorState;
using rsim_plugin::ActorUpdate;
using rsim_plugin::IPluginController;
using rsim_plugin::TickContext;

namespace
{

    constexpr double kFallbackDeceleration = 6.0;
    constexpr double kMinTimeStep = 1e-6;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTrajectoryUpdateLookahead = 0.05;

    double NormalizeAngle(double angle)
    {
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle < -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    double InterpolateAngle(double from, double to, double ratio)
    {
        return NormalizeAngle(from + NormalizeAngle(to - from) * ratio);
    }

    bool IsValidTrajectoryPoint(
        const rsim_driver::PlanningTrajectoryPoint &point)
    {
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.heading) &&
               std::isfinite(point.curvature) &&
               std::isfinite(point.speed) &&
               point.speed >= 0.0 &&
               std::isfinite(point.accel) &&
               std::isfinite(point.time);
    }

    rsim_driver::PlanningTrajectoryPoint InterpolateTrajectoryPoint(
        const rsim_driver::PlanningTrajectoryPoint &previous,
        const rsim_driver::PlanningTrajectoryPoint &next,
        double time)
    {
        const double dt = next.time - previous.time;
        const double ratio = dt > kMinTimeStep
                                 ? std::clamp((time - previous.time) / dt,
                                              0.0,
                                              1.0)
                                 : 0.0;

        rsim_driver::PlanningTrajectoryPoint point;
        point.x = previous.x + (next.x - previous.x) * ratio;
        point.y = previous.y + (next.y - previous.y) * ratio;
        point.heading = InterpolateAngle(previous.heading, next.heading, ratio);
        point.curvature =
            previous.curvature + (next.curvature - previous.curvature) * ratio;
        point.speed = previous.speed + (next.speed - previous.speed) * ratio;
        point.accel = previous.accel + (next.accel - previous.accel) * ratio;
        point.time = time;
        return point;
    }

    bool FindTrajectoryPointAtTime(
        const std::vector<rsim_driver::PlanningTrajectoryPoint> &trajectory,
        double queryTime,
        rsim_driver::PlanningTrajectoryPoint *point,
        std::size_t *targetIdx)
    {
        if (point == nullptr || targetIdx == nullptr ||
            trajectory.empty() || !std::isfinite(queryTime))
        {
            return false;
        }

        for (std::size_t i = 0; i < trajectory.size(); ++i)
        {
            const auto &trajectoryPoint = trajectory[i];
            if (!IsValidTrajectoryPoint(trajectoryPoint))
                return false;
            if (i > 0 &&
                trajectoryPoint.time < trajectory[i - 1].time - kMinTimeStep)
            {
                return false;
            }
        }

        if (trajectory.size() == 1)
        {
            if (std::fabs(queryTime - trajectory.front().time) > kMinTimeStep)
                return false;
            *point = trajectory.front();
            point->time = queryTime;
            *targetIdx = 0;
            return true;
        }

        if (queryTime < trajectory.front().time - kMinTimeStep ||
            queryTime > trajectory.back().time + kMinTimeStep)
        {
            return false;
        }

        if (queryTime <= trajectory.front().time + kMinTimeStep)
        {
            *point = trajectory.front();
            point->time = queryTime;
            *targetIdx = 0;
            return true;
        }

        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const auto &previous = trajectory[i - 1];
            const auto &current = trajectory[i];
            if (current.time < previous.time - kMinTimeStep)
                return false;
            if (queryTime > current.time + kMinTimeStep)
                continue;

            if (queryTime < previous.time - kMinTimeStep)
                return false;

            *point = InterpolateTrajectoryPoint(previous, current, queryTime);
            *targetIdx = i;
            return IsValidTrajectoryPoint(*point);
        }

        *point = trajectory.back();
        point->time = queryTime;
        *targetIdx = trajectory.size() - 1;
        return true;
    }

    class RSimDriverPlugin : public IPluginController
    {
    public:
        ~RSimDriverPlugin() override
        {
            if (reference_line_csv_fp_ != nullptr)
                std::fclose(reference_line_csv_fp_);
            if (planning_start_sl_csv_fp_ != nullptr)
                std::fclose(planning_start_sl_csv_fp_);
            if (ego_trajectory_csv_fp_ != nullptr)
                std::fclose(ego_trajectory_csv_fp_);
        }

        // ========================================================================
        // Init() — 插件初始化: 加载地图 + 解析 XOSC 全局路径
        // ========================================================================
        void Init(const std::map<std::string, std::string> &properties,
                  const std::vector<int32_t> &controlled_actor_ids) override
        {
            controlled_ids_ = controlled_actor_ids;

            auto getStr = [&](const char *key, const char *def)
            {
                auto it = properties.find(key);
                return (it != properties.end()) ? it->second : def;
            };
            auto getDouble = [&](const char *key, double def) -> double
            {
                auto it = properties.find(key);
                if (it == properties.end())
                    return def;
                try
                {
                    return std::stod(it->second);
                }
                catch (...)
                {
                    return def;
                }
            };
            std::string xodr_path = getStr("xodrPath", "");
            route_xosc_path_ = getStr("routeXoscPath", "");
            route_csv_path_ = getStr("routeCsvPath", "");
            reference_line_csv_path_ = getStr("referenceLineCsvPath", "");
            obstacle_csv_path_ = getStr("obstacleCsvPath", "");
            planning_start_sl_csv_path_ = getStr("planningStartSlCsvPath", "");
            ego_trajectory_csv_path_ = getStr("egoTrajectoryCsvPath", "");
            entity_name_ = getStr("entityName", "ego");
            set_speed_ = getDouble("setSpeed", 13.0);
            rsim_driver::EmPlannerConfig emPlannerConfig = em_planner_.config();
            emPlannerConfig.speed_config.reference_speed = set_speed_;
            em_planner_.SetConfig(emPlannerConfig);
            OpenReferenceLineDebugCsv();
            OpenPlanningStartSlDebugCsv();
            OpenEgoTrajectoryCsv();
            obstacle_csv_writer_.Open(obstacle_csv_path_);

            // ---- 加载 OpenDRIVE 地图 (用于 lane/track 查询) ----
            if (xodr_path.empty() || !map_.Load(xodr_path))
            {
                std::fprintf(stderr,
                             "[RSimDriver] FATAL: xodrPath 属性缺失或地图加载失败 ('%s')\n",
                             xodr_path.c_str());
                map_loaded_ = false;
            }
            else
            {
                map_loaded_ = true;
            }

            // ---- 从 runtime XOSC 读取 ego FollowTrajectoryAction ----
            bool xosc_route_installed = false;
            if (map_loaded_ && InstallGlobalPathFromXosc(route_xosc_path_))
            {
                xosc_route_installed = true;
            }

            /*std::fprintf(stderr,
                         "[RSimDriver] ========================================================\n"
                         "[RSimDriver] Init done:\n"
                         "[RSimDriver]   xodrPath          = %s\n"
                         "[RSimDriver]   routeXoscPath     = %s\n"
                         "[RSimDriver]   entityName        = %s\n"
                         "[RSimDriver]   setSpeed          = %.2f m/s\n"
                         "[RSimDriver]   refLineCsvPath    = %s\n"
                         "[RSimDriver]   mapLoaded          = %s\n"
                         "[RSimDriver]   routeInstalled     = %s\n"
                         "[RSimDriver]   routeSegments      = %zu\n"
                         "[RSimDriver]   worldPoints        = %zu\n"
                         "[RSimDriver] ========================================================\n",
                         xodr_path.c_str(),
                         route_xosc_path_.empty() ? "(none)" : route_xosc_path_.c_str(),
                         entity_name_.c_str(),
                         set_speed_,
                         reference_line_csv_path_.empty() ? "(none)" : reference_line_csv_path_.c_str(),
                         map_loaded_ ? "ok" : "FAILED",
                         xosc_route_installed ? "yes" : "no",
                         route_segments_.size(),
                         global_path_world_points_.size());
                         */
        }

        // ========================================================================
        // Step() — 每帧调用: EMPlanner → FrenetToCartesian → ActorUpdate
        // ========================================================================
        std::vector<ActorUpdate> Step(const TickContext &ctx) override
        {
            std::vector<ActorUpdate> updates;
            if (controlled_ids_.empty())
                return updates;

            const ActorState *ego = FindControlledActor(ctx);
            if (ego == nullptr)
                return updates;

            if (!map_loaded_)
            {
                ReportPlanningFailure(ctx, *ego, "map is not loaded");
                updates.push_back(BuildStopActorUpdate(*ego));
                return updates;
            }

            obstacle_csv_writer_.WriteFrame(ctx.frame_id,
                                            ctx.sim_time,
                                            *ego,
                                            ctx.actors,
                                            controlled_ids_);

            LatchInitialState(ego);
            UpdateReferenceLine(*ego);

            if (reference_line_ == nullptr || reference_line_->points.empty())
            {
                ReportPlanningFailure(ctx, *ego, "reference line is empty");
                updates.push_back(BuildStopActorUpdate(*ego));
                return updates;
            }

            rsim_driver::EmPlannerResult plannerResult;
            if (!RunEmPlannerDetailed(ctx, *ego, &plannerResult))
            {
                CachePlannerResult(plannerResult);
                if (plannerResult.planning_start_success)
                {
                    WritePlanningStartSlDebugCsv(ctx,
                                                 *ego,
                                                 plannerResult.planning_start_result,
                                                 plannerResult.frenet_start_result,
                                                 plannerResult.frenet_start_success);
                }
                ReportPlannerStageStatus(ctx, plannerResult);
                ReportPlanningFailure(ctx, *ego, "EM planner failed");
                updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
                return updates;
            }
            CachePlannerResult(plannerResult);
            WritePlanningStartSlDebugCsv(ctx,
                                         *ego,
                                         plannerResult.planning_start_result,
                                         plannerResult.frenet_start_result,
                                         plannerResult.frenet_start_success);

            if (plannerResult.dp_result.fallback == rsim_driver::DpFallback::Stop)
            {
                ReportPlannerStageStatus(ctx, plannerResult);
                updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
                return updates;
            }

            if (plannerResult.trajectory.empty())
            {
                ReportPlannerStageStatus(ctx, plannerResult);
                ReportPlanningFailure(ctx, *ego, "EM planner trajectory is empty");
                updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
                return updates;
            }
            std::size_t target_idx = 0;
            rsim_driver::PlanningTrajectoryPoint target;
            const double target_time = ctx.sim_time + kTrajectoryUpdateLookahead;
            if (!FindTrajectoryPointAtTime(plannerResult.trajectory,
                                           target_time,
                                           &target,
                                           &target_idx))
            {
                ReportPlannerStageStatus(ctx, plannerResult);
                ReportPlanningFailure(ctx,
                                      *ego,
                                      "EM planner trajectory does not cover update time");
                updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
                return updates;
            }

            WriteEgoTrajectoryCsv(ctx,
                                  *ego,
                                  target_idx,
                                  plannerResult.localfrenetpath,
                                  plannerResult.trajectory);
            updates.push_back(BuildActorUpdateFromTrajectoryPoint(*ego, target));

            rsim_driver::CartesianPathPoint debugTarget;
            debugTarget.x = target.x;
            debugTarget.y = target.y;
            debugTarget.heading = target.heading;
            debugTarget.kappa = target.curvature;
            WriteReferenceLineDebugCsv(ctx, *ego, target_idx, debugTarget);

            return updates;
        }

    private:
        bool RunEmPlannerDetailed(
            const TickContext &ctx,
            const ActorState &ego,
            rsim_driver::EmPlannerResult *result)
        {
            if (result == nullptr || reference_line_ == nullptr)
                return false;

            rsim_driver::EmPlannerResult output;
            if (!em_planner_.EMPlanPathDetailed(ctx.actors,
                                                ego.id,
                                                ego,
                                                ctx.sim_time,
                                                previous_trajectory_,
                                                reference_line_->points,
                                                &output))
            {
                *result = std::move(output);
                return false;
            }

            // DP blocked: skip speed planning, let vehicle hold position
            if (output.dp_result.fallback == rsim_driver::DpFallback::Stop)
            {
                *result = std::move(output);
                return true;
            }

            if (!em_planner_.EMPlanSpeedDetailed(
                    ctx.actors,
                    ego.id,
                    output.planning_start_result,
                    output.qp_result,
                    &output))
            {
                *result = std::move(output);
                return false;
            }

            if (!em_planner_.EMPlanPostProcessDetailed(output.planning_start_result,
                                                       output.qp_result,
                                                       output.speed_result,
                                                       &output.trajectory))
            {
                output.trajectory_success = false;
                *result = std::move(output);
                return false;
            }
            output.trajectory_success = true;

            *result = std::move(output);
            return true;
        }

        const char *PlanningStartSourceName(
            rsim_driver::PlanningStartSource source) const
        {
            switch (source)
            {
            case rsim_driver::PlanningStartSource::KinematicExtrapolation:
                return "KinematicExtrapolation";
            case rsim_driver::PlanningStartSource::PreviousTrajectory:
                return "PreviousTrajectory";
            }
            return "Unknown";
        }

        void CachePlannerResult(const rsim_driver::EmPlannerResult &result)
        {
            static_frenet_obstacles_ = result.static_perception_result;
            dynamic_frenet_obstacles_ = result.dynamic_perception_result;
            frenet_obstacles_valid_ = result.perception_success;
            planning_start_result_ = result.planning_start_result;
            planning_start_frenet_ = result.frenet_start_result;
            planning_start_frenet_valid_ = result.frenet_start_success;
            dp_planning_result_ = result.dp_result;
            localfrenetpath_ = result.localfrenetpath;
            cartesian_plan_path_ = result.qp_result.localcartesianpath;
            planned_trajectory_ = result.trajectory;

            if (result.trajectory_success)
            {
                previous_trajectory_ = result.trajectory;
            }
        }

        const char *PlannerFailureStage(
            const rsim_driver::EmPlannerResult &result) const
        {
            if (!result.static_perception_success)
                return "static_perception";
            if (!result.dynamic_perception_success)
                return "dynamic_perception";
            if (!result.planning_start_success)
                return "planning_start";
            if (!result.frenet_start_success)
                return "frenet_start";
            if (!result.dp_success)
                return "dp";
            if (!result.drivable_area_success)
                return "drivable_area";
            if (!result.qp_success)
                return "qp";
            if (result.speed_reference_line.empty())
                return "speed_reference_line";
            if (!result.speed_success)
                return "speed";
            if (!result.trajectory_success)
                return "trajectory";
            return "unknown";
        }

        void ReportPlannerStageStatus(
            const TickContext &ctx,
            const rsim_driver::EmPlannerResult &result) const
        {
            std::fprintf(stderr,
                         "[RSimDriver] EM planner stages frame=%llu time=%.6f "
                         "first_failed=%s static=%d dynamic=%d start=%d frenet=%d "
                         "dp=%d stop=%d drivable=%d qp=%d speed_ref=%zu speed=%d trajectory=%d "
                         "static_obs=%zu virtual_obs=%zu dynamic_obs=%zu "
                         "virtual_seeds=%zu dp_points=%zu qp_points=%zu "
                         "speed_points=%zu trajectory_points=%zu previous_points=%zu\n",
                         static_cast<unsigned long long>(ctx.frame_id),
                         ctx.sim_time,
                         PlannerFailureStage(result),
                         result.static_perception_success ? 1 : 0,
                         result.dynamic_perception_success ? 1 : 0,
                         result.planning_start_success ? 1 : 0,
                         result.frenet_start_success ? 1 : 0,
                         result.dp_success ? 1 : 0,
                         result.dp_result.fallback == rsim_driver::DpFallback::Stop ? 1 : 0,
                         result.drivable_area_success ? 1 : 0,
                         result.qp_success ? 1 : 0,
                         result.speed_reference_line.size(),
                         result.speed_success ? 1 : 0,
                         result.trajectory_success ? 1 : 0,
                         result.static_perception_result.staticobstacles.size(),
                         result.virtual_static_obstacles.size(),
                         result.dynamic_perception_result.dynamicobstacles.size(),
                         result.virtual_obstacle_seeds.size(),
                         result.dp_result.path.size(),
                         result.qp_result.localcartesianpath.size(),
                         result.speed_result.stpoints.size(),
                         result.trajectory.size(),
                         previous_trajectory_.size());
        }

        void ReportPlanningFailure(const TickContext &ctx,
                                   const ActorState &ego,
                                   const char *reason) const
        {
            std::fprintf(stderr,
                         "[RSimDriver] ERROR: %s, stop ego id=%d frame=%llu time=%.6f "
                         "ego=(%.3f, %.3f, h=%.3f)\n",
                         reason,
                         ego.id,
                         static_cast<unsigned long long>(ctx.frame_id),
                         ctx.sim_time,
                         ego.x,
                         ego.y,
                         ego.h);
        }

        bool InstallGlobalPathFromXosc(const std::string &xoscPath)
        {
            rsim_driver::GlobalPathResult result;
            if (!global_path_generator_.GenerateFromXosc(
                    xoscPath, entity_name_, map_, &result))
            {
                return false;
            }

            route_segments_ = std::move(result.route_segments);
            global_path_world_points_.clear();
            global_path_world_points_.reserve(result.world_points.size());
            for (const auto &point : result.world_points)
            {
                rsim_driver::WorldPoint worldPoint;
                worldPoint.x = point.x;
                worldPoint.y = point.y;
                global_path_world_points_.push_back(worldPoint);
            }

            closest_global_path_idx_ = 0;
            const bool csvWritten =
                global_path_generator_.WriteCsv(route_csv_path_, result.world_points);

            if (csvWritten && !route_csv_path_.empty())
            {
                std::fprintf(stderr,
                             "[RSimDriver]###INIT运行 global_path CSV written: %s (rows=%zu, chord=%.2f m)\n",
                             route_csv_path_.c_str(),
                             global_path_world_points_.size(),
                             result.chord_length);
            }
            return true;
        }

        // 从 TickContext 中查找本插件控制的第一个 actor
        const ActorState *FindControlledActor(const TickContext &ctx) const
        {
            for (const auto &a : ctx.actors)
                if (a.id == controlled_ids_.front())
                    return &a;
            return nullptr;
        }

        ActorUpdate BuildStopActorUpdate(const ActorState &ego) const
        {
            ActorUpdate update;
            update.actor_id = ego.id;
            update.x = ego.x;
            update.y = ego.y;
            update.z = ego.z;
            update.h = ego.h;
            update.p = ego.p;
            update.r = ego.r;
            update.speed = 0.0;
            update.wheel_angle = 0.0;
            update.position_valid = 1;
            update.speed_valid = 1;
            return update;
        }

        ActorUpdate BuildControlledStopActorUpdate(
            const ActorState &ego,
            double dt) const
        {
            ActorUpdate update;
            update.actor_id = ego.id;

            const double timeStep = dt > kMinTimeStep ? dt : 0.0;
            const double speed =
                std::isfinite(ego.speed) ? std::max(0.0, ego.speed) : 0.0;
            const double nextSpeed =
                std::max(0.0, speed - kFallbackDeceleration * timeStep);
            const double distance = 0.5 * (speed + nextSpeed) * timeStep;
            const double heading = std::isfinite(ego.h) ? ego.h : 0.0;

            update.x = ego.x + distance * std::cos(heading);
            update.y = ego.y + distance * std::sin(heading);
            update.z = ego.z;
            update.h = heading;
            update.p = ego.p;
            update.r = ego.r;
            update.speed = nextSpeed;
            update.wheel_angle = 0.0;
            update.position_valid = 1;
            update.speed_valid = 1;
            return update;
        }

        // 使用目标时间的规划轨迹点直接更新 SceneRunner controller 状态。
        ActorUpdate BuildActorUpdateFromTrajectoryPoint(
            const ActorState &ego,
            const rsim_driver::PlanningTrajectoryPoint &target) const
        {
            ActorUpdate update;
            update.actor_id = ego.id;
            update.x = target.x;
            update.y = target.y;
            update.z = ego.z;
            update.h = target.heading;
            update.p = ego.p;
            update.r = ego.r;
            update.speed = target.speed;
            update.wheel_angle = 0.0;
            update.position_valid = 1;
            update.speed_valid = 1;
            return update;
        }

        void OpenReferenceLineDebugCsv()
        {
            if (reference_line_csv_fp_ != nullptr)
            {
                std::fclose(reference_line_csv_fp_);
                reference_line_csv_fp_ = nullptr;
            }

            if (reference_line_csv_path_.empty())
                return;

            reference_line_csv_fp_ = std::fopen(reference_line_csv_path_.c_str(), "w");
            if (reference_line_csv_fp_ == nullptr)
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: 无法写入参考线调试 CSV: %s\n",
                             reference_line_csv_path_.c_str());
                reference_line_csv_path_.clear();
                return;
            }

            std::fprintf(reference_line_csv_fp_,
                         "frame_id,sim_time,ego_x,ego_y,global_match_idx,"
                         "projection_x,projection_y,projection_hdg,"
                         "target_idx,point_idx,ref_s,ref_x,ref_y,ref_hdg,target_x,target_y\n");
            std::fflush(reference_line_csv_fp_);
        }

        void WriteReferenceLineDebugCsv(const TickContext &ctx,
                                        const ActorState &ego,
                                        std::size_t targetIdx,
                                        const rsim_driver::CartesianPathPoint &target)
        {
            if (reference_line_csv_fp_ == nullptr || reference_line_ == nullptr)
                return;

            const rsim_driver::ReferencePoint projectionPoint =
                reference_line_generator_.curProjectionPoint();
            for (std::size_t i = 0; i < reference_line_->points.size(); ++i)
            {
                const rsim_driver::ReferencePoint &point = reference_line_->points[i];
                std::fprintf(reference_line_csv_fp_,
                             "%llu,%.9f,%.9f,%.9f,%zu,"
                             "%.9f,%.9f,%.9f,"
                             "%zu,%zu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                             static_cast<unsigned long long>(ctx.frame_id),
                             ctx.sim_time,
                             ego.x,
                             ego.y,
                             reference_line_generator_.lastMatchPointIndex(),
                             projectionPoint.x,
                             projectionPoint.y,
                             projectionPoint.hdg,
                             targetIdx,
                             i,
                             point.s,
                             point.x,
                             point.y,
                             point.hdg,
                             target.x,
                             target.y);
            }
            std::fflush(reference_line_csv_fp_);
        }

        void OpenPlanningStartSlDebugCsv()
        {
            if (planning_start_sl_csv_fp_ != nullptr)
            {
                std::fclose(planning_start_sl_csv_fp_);
                planning_start_sl_csv_fp_ = nullptr;
            }

            if (planning_start_sl_csv_path_.empty())
                return;

            planning_start_sl_csv_fp_ =
                std::fopen(planning_start_sl_csv_path_.c_str(), "w");
            if (planning_start_sl_csv_fp_ == nullptr)
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: 无法写入规划起点 Frenet 调试 CSV: %s\n",
                             planning_start_sl_csv_path_.c_str());
                planning_start_sl_csv_path_.clear();
                return;
            }

            std::fprintf(planning_start_sl_csv_fp_,
                         "frame_id,sim_time,time_step,"
                         "ego_x,ego_y,ego_h,ego_speed,ego_acc_x,ego_acc_y,ego_accel,"
                         "start_x,start_y,start_heading,start_speed,start_accel,start_time,"
                         "start_source,match_distance,start_curvature,"
                         "sl_success,s,l,l_prime,l_double_prime,"
                         "previous_trajectory_size,stitching_trajectory_size\n");
            std::fflush(planning_start_sl_csv_fp_);
        }

        void WritePlanningStartSlDebugCsv(
            const TickContext &ctx,
            const ActorState &ego,
            const rsim_driver::PlanningStartResult &startResult,
            const rsim_driver::CartesianFrenetState &frenet,
            bool slSuccess)
        {
            if (planning_start_sl_csv_fp_ == nullptr)
                return;

            const rsim_driver::PlanningStartPoint &start = startResult.start_point;
            const double egoAccel =
                //std::sqrt(ego.acc_x * ego.acc_x + ego.acc_y * ego.acc_y)
                ego.acc_x;
            std::fprintf(planning_start_sl_csv_fp_,
                         "%llu,%.9f,%.9f,"
                         "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                         "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                         "%s,%.9f,%.9f,"
                         "%d,%.9f,%.9f,%.9f,%.9f,"
                         "%zu,%zu\n",
                         static_cast<unsigned long long>(ctx.frame_id),
                         ctx.sim_time,
                         ctx.time_step,
                         ego.x,
                         ego.y,
                         ego.h,
                         ego.speed,
                         ego.acc_x,
                         ego.acc_y,
                         egoAccel,
                         start.x,
                         start.y,
                         start.heading,
                         start.speed,
                         start.accel,
                         start.time,
                         PlanningStartSourceName(start.source),
                         start.matchDistance,
                         startResult.start_curvature,
                         slSuccess ? 1 : 0,
                         frenet.s,
                         // frenet.s_dot,
                         // frenet.s_ddot,
                         frenet.l,
                         frenet.l_prime,
                         frenet.l_double_prime,
                         previous_trajectory_.size(),
                         startResult.stitching_trajectory.size());
            std::fflush(planning_start_sl_csv_fp_);
        }

        void OpenEgoTrajectoryCsv()
        {
            if (ego_trajectory_csv_fp_ != nullptr)
            {
                std::fclose(ego_trajectory_csv_fp_);
                ego_trajectory_csv_fp_ = nullptr;
            }

            if (ego_trajectory_csv_path_.empty())
                return;

            ego_trajectory_csv_fp_ =
                std::fopen(ego_trajectory_csv_path_.c_str(), "w");
            if (ego_trajectory_csv_fp_ == nullptr)
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: cannot write ego trajectory CSV: %s\n",
                             ego_trajectory_csv_path_.c_str());
                ego_trajectory_csv_path_.clear();
                return;
            }

            std::fprintf(ego_trajectory_csv_fp_,
                         "frame_id,sim_time,time_step,ego_x,ego_y,ego_h,"
                         "target_idx,point_idx,s,l,l_prime,l_double_prime,"
                         "x,y,theta,k,v,a,time,is_target\n");
            std::fflush(ego_trajectory_csv_fp_);
        }

        void WriteEgoTrajectoryCsv(
            const TickContext &ctx,
            const ActorState &ego,
            std::size_t targetIdx,
            const std::vector<rsim_driver::DpPathPoint> &localfrenetpath,
            const std::vector<rsim_driver::PlanningTrajectoryPoint> &path)
        {
            if (ego_trajectory_csv_fp_ == nullptr)
                return;

            const double nan = std::numeric_limits<double>::quiet_NaN();
            for (std::size_t i = 0; i < path.size(); ++i)
            {
                const rsim_driver::DpPathPoint *frenetPoint =
                    i < localfrenetpath.size() ? &localfrenetpath[i] : nullptr;
                const rsim_driver::PlanningTrajectoryPoint &point = path[i];
                std::fprintf(ego_trajectory_csv_fp_,
                             "%llu,%.9f,%.9f,%.9f,%.9f,%.9f,"
                             "%zu,%zu,%.9f,%.9f,%.9f,%.9f,"
                             "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d\n",
                             static_cast<unsigned long long>(ctx.frame_id),
                             ctx.sim_time,
                             ctx.time_step,
                             ego.x,
                             ego.y,
                             ego.h,
                             targetIdx,
                             i,
                             frenetPoint != nullptr ? frenetPoint->s : nan,
                             frenetPoint != nullptr ? frenetPoint->l : nan,
                             frenetPoint != nullptr ? frenetPoint->l_prime : nan,
                             frenetPoint != nullptr ? frenetPoint->l_double_prime : nan,
                             point.x,
                             point.y,
                             point.heading,
                             point.curvature,
                             point.speed,
                             point.accel,
                             point.time,
                             i == targetIdx ? 1 : 0);
            }
            std::fflush(ego_trajectory_csv_fp_);
        }

        // ========================================================================
        // LatchInitialState() — 首次运行时绑定 ego 到全局路径起点
        // ========================================================================
        void LatchInitialState(const ActorState *&ego)
        {
            if (latched_)
                return;

            current_speed_ = ego->speed;
            current_s_ = ego->s;
            current_road_ = ego->road_id;
            current_lane_ = ego->lane_id;

            if (!route_segments_.empty())
            {
                const auto &first = route_segments_.front();
                current_road_ = first.road_id;
                current_s_ = first.s_start;
                if (first.lane_id != 0)
                    current_lane_ = first.lane_id;
                else
                    current_lane_ = map_.TrackTToLane(first.road_id, first.s_start, first.t);
                closest_global_path_idx_ = 0;

                const double dx0 = global_path_world_points_.empty()
                                       ? 0.0
                                       : ego->x - global_path_world_points_.front().x;
                const double dy0 = global_path_world_points_.empty()
                                       ? 0.0
                                       : ego->y - global_path_world_points_.front().y;
                const double sceneRunnerStartError = std::sqrt(dx0 * dx0 + dy0 * dy0);

                if (sceneRunnerStartError > 2.0)
                {
                    std::fprintf(stderr,
                                 "[RSimDriver] WARNING: SceneRunner 初始 ego (%.2f, %.2f) "
                                 "距离全局路径起点 %.2f m, 插件内部起点已锚到 route start\n",
                                 ego->x, ego->y, sceneRunnerStartError);
                }

                std::fprintf(stderr,
                             "[RSimDriver]###INIT运行    LatchInitialState: 主车（teleportaction）ego scene=(%.2f, %.2f)⬇\n"
                             "[RSimDriver]全局路径起点   route start seg[0] road=%lld  s=%.2f（路段）, "
                             "worldPt[0] (%.2f, %.2f)\n[RSimDriver]车辆初始点（scenerunner）与全局路径起点（RsimDriverplugin）一致\n",
                             ego->x, ego->y,
                             static_cast<long long>(current_road_),
                             current_s_,
                             global_path_world_points_.empty() ? 0.0
                                                               : global_path_world_points_.front().x,
                             global_path_world_points_.empty() ? 0.0
                                                               : global_path_world_points_.front().y);
            }

            latched_ = true;
        }

        // ========================================================================
        // UpdateReferenceLine() — 基于 ego 当前世界坐标生成局部平滑参考线
        // ========================================================================
        void UpdateReferenceLine(const ActorState &ego)
        {
            if (global_path_world_points_.empty())
                return;

            std::unique_ptr<rsim_driver::ReferenceLine> generated =
                reference_line_generator_.Generate(global_path_world_points_, ego.x, ego.y);
            if (!generated)
            {
                reference_line_.reset();
                if (!reference_line_failure_reported_)
                {
                    std::fprintf(stderr,
                                 "[RSimDriver] WARNING: reference line generation failed "
                                 "(worldPoints=%zu)\n",
                                 global_path_world_points_.size());
                    reference_line_failure_reported_ = true;
                }
                return;
            }

            reference_line_ = std::move(generated);
            reference_line_failure_reported_ = false;

            if (!reference_line_ready_reported_)
            {

                std::fprintf(stderr,
                             "[RSimDriver]###初始运行 Reference line scale: points=%zu \n",
                             reference_line_->points.size());
                reference_line_ready_reported_ = true;
            }
            // std::printf("globalMatchIdx=%zu \n", reference_line_generator_.lastMatchPointIndex());
        }

        // ========================================================================
        // 成员变量
        // ========================================================================
        std::vector<int32_t> controlled_ids_;
        rsim_driver::MapHelper map_;

        // ---- 全局路由 — 路段索引 (用于 road/lane tracking) ----
        std::vector<rsim_driver::GlobalPathRouteSegment> route_segments_;

        // ---- 全局路由 — 世界坐标点 (供下游规划模块构建参考线) ----
        std::vector<rsim_driver::WorldPoint> global_path_world_points_;
        size_t closest_global_path_idx_ = 0;

        // ---- 局部参考线 ----
        rsim_driver::ReferenceLineGenerator reference_line_generator_;
        std::unique_ptr<rsim_driver::ReferenceLine> reference_line_;
        double current_reference_s_ = 0.0;
        bool reference_line_ready_reported_ = false;
        bool reference_line_failure_reported_ = false;

        // ---- XOSC route ----
        std::string entity_name_ = "ego";
        std::string route_xosc_path_;
        std::string route_csv_path_;
        std::string reference_line_csv_path_;
        std::string obstacle_csv_path_;
        std::string planning_start_sl_csv_path_;
        std::string ego_trajectory_csv_path_;
        std::FILE *reference_line_csv_fp_ = nullptr;
        std::FILE *planning_start_sl_csv_fp_ = nullptr;
        std::FILE *ego_trajectory_csv_fp_ = nullptr;
        rsim_driver::GlobalPathGenerator global_path_generator_;
        rsim_driver::ObstacleCsvWriter obstacle_csv_writer_;

        // ---- EM planner pipeline ----
        rsim_driver::EmPlanner em_planner_;
        rsim_driver::StaticFrenetObstaclePerceptionResult static_frenet_obstacles_;
        rsim_driver::DynamicFrenetObstaclePerceptionResult dynamic_frenet_obstacles_;
        bool frenet_obstacles_valid_ = false;
        std::vector<rsim_driver::PlanningTrajectoryPoint> previous_trajectory_;
        rsim_driver::PlanningStartResult planning_start_result_;
        rsim_driver::CartesianFrenetState planning_start_frenet_;
        bool planning_start_frenet_valid_ = false;
        rsim_driver::DpPlannerResult dp_planning_result_;
        std::vector<rsim_driver::DpPathPoint> localfrenetpath_;
        std::vector<rsim_driver::CartesianPathPoint> cartesian_plan_path_;
        std::vector<rsim_driver::PlanningTrajectoryPoint> planned_trajectory_;

        // ---- ego 初始状态 ----
        double set_speed_ = 8.0;
        double current_speed_ = 0.0;
        double current_s_ = 0.0;
        int32_t current_road_ = 0;
        int current_lane_ = 0;
        bool latched_ = false;
        bool map_loaded_ = false;
    };

} // namespace

// 插件工厂入口: 由仿真器动态加载 .so 时调用
extern "C" IPluginController *CreateController(const char *name)
{
    (void)name;
    return new RSimDriverPlugin();
}

// 插件析构入口: 由仿真器卸载 .so 时调用
extern "C" void DestroyController(IPluginController *p)
{
    delete p;
}
