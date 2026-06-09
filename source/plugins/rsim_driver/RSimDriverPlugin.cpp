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
 *     └── 预留规划扩展点 (当前为空壳)
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
 *   pointStep    : 每帧沿当前参考线向前推进的离散点数 (默认 2)
 *   referenceLineCsvPath: 参考线运动调试 CSV 输出路径 (可选)
 *   obstacleCsvPath: 障碍物转化 CSV 输出路径 (可选)
 * ============================================================================
 */

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include "GlobalPathGenerator.hpp"
#include "MapHelper.hpp"
#include "ObstacleToCsv.hpp"
#include "ReferenceLineGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
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

    class RSimDriverPlugin : public IPluginController
    {
    public:
        ~RSimDriverPlugin() override
        {
            if (reference_line_csv_fp_ != nullptr)
                std::fclose(reference_line_csv_fp_);
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
            auto getInt = [&](const char *key, int def) -> int
            {
                auto it = properties.find(key);
                if (it == properties.end())
                    return def;
                try
                {
                    return std::stoi(it->second);
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
            entity_name_ = getStr("entityName", "ego");
            set_speed_ = getDouble("setSpeed", 10.0);
            point_step_ = std::max(1, getInt("pointStep", 2));
            OpenReferenceLineDebugCsv();
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
                         "[RSimDriver]   pointStep         = %d\n"
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
                         point_step_,
                         reference_line_csv_path_.empty() ? "(none)" : reference_line_csv_path_.c_str(),
                         map_loaded_ ? "ok" : "FAILED",
                         xosc_route_installed ? "yes" : "no",
                         route_segments_.size(),
                         global_path_world_points_.size());
                         */
        }

        // ========================================================================
        // Step() — 每帧调用 (当前为空壳, 预留规划扩展点)
        // ========================================================================
        std::vector<ActorUpdate> Step(const TickContext &ctx) override
        {
            std::vector<ActorUpdate> updates;
            if (!map_loaded_ || controlled_ids_.empty())
                return updates;

            const ActorState *ego = FindControlledActor(ctx);
            if (ego == nullptr)
                return updates;

            obstacle_csv_writer_.WriteFrame(ctx.frame_id,
                                            ctx.sim_time,
                                            *ego,
                                            ctx.actors,
                                            controlled_ids_);

            LatchInitialState(ego);
            UpdateReferenceLine(*ego);

            if (reference_line_ == nullptr || reference_line_->points.empty())
                return updates;

            const std::size_t target_idx =
                FindForwardReferencePointIndex(*reference_line_, point_step_);
            const rsim_driver::ReferencePoint &target = reference_line_->points[target_idx];

            updates.push_back(BuildActorUpdateFromReferencePoint(*ego, target, ctx.time_step));
            WriteReferenceLineDebugCsv(ctx, *ego, target_idx, target);

            return updates;
        }

    private:
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

        // 在 signed s 参考线上, 从 s=0 后第一个正向点开始数 pointStep 个点作为目标点
        std::size_t FindForwardReferencePointIndex(const rsim_driver::ReferenceLine &line,
                                                   int pointStep) const
        {
            if (line.points.empty())
                return 0;

            std::size_t firstForward = line.points.size();
            for (std::size_t i = 0; i < line.points.size(); ++i)
            {
                if (line.points[i].s > 1e-6)
                {
                    firstForward = i;
                    break;
                }
            }

            if (firstForward >= line.points.size())
                return line.points.size() - 1;

            const std::size_t step = static_cast<std::size_t>(std::max(1, pointStep));
            return std::min(firstForward + step - 1, line.points.size() - 1);
        }

        // 将参考线上的目标点转换成 SceneRunner controller 更新
        ActorUpdate BuildActorUpdateFromReferencePoint(
            const ActorState &ego,
            const rsim_driver::ReferencePoint &target,
            double dt) const
        {
            ActorUpdate update;
            update.actor_id = ego.id;
            update.x = target.x;
            update.y = target.y;
            update.z = ego.z;
            update.h = target.hdg;
            update.p = ego.p;
            update.r = ego.r;

            const double dx = target.x - ego.x;
            const double dy = target.y - ego.y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            update.speed = (dt > 1e-6) ? distance / dt : 0.0;
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
                                        const rsim_driver::ReferencePoint &target)
        {
            if (reference_line_csv_fp_ == nullptr || reference_line_ == nullptr)
                return;

            const rsim_driver::ReferencePoint projectionPoint =
                reference_line_generator_.lastProjectionPoint();
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
        std::FILE *reference_line_csv_fp_ = nullptr;
        rsim_driver::GlobalPathGenerator global_path_generator_;
        rsim_driver::ObstacleCsvWriter obstacle_csv_writer_;

        // ---- ego 初始状态 ----
        double set_speed_ = 10.0;
        int point_step_ = 2;
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
