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
 *     └── 解析 routeXoscPath 指向的 runtime XOSC
 *            └── Init/Private[@entityRef="ego"]/FollowTrajectoryAction
 *            │
 *            ▼
 *   RSimDriverPlugin::Step() (每帧)
 *     └── 预留规划扩展点 (当前为空壳)
 *
 * ---- 全局路径数据结构 ----
 *
 *   XOSC FollowTrajectoryAction / Polyline / Vertex:
 *     ├── RoadPosition: roadId, s, t
 *     └── WorldPosition: x, y
 *
 *   插件内部两份数据:
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

#include "pugixml.hpp"

#include "MapHelper.hpp"
#include "ObstacleToCsv.hpp"
#include "ReferenceLineGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

    // 判断 XML 节点名称是否匹配 (pugixml 无此内置方法)
    bool NodeNameIs(const pugi::xml_node &node, const char *name)
    {
        return std::strcmp(node.name(), name) == 0;
    }

    // 深度优先搜索第一个匹配标签名的后代节点
    pugi::xml_node FindFirstDescendant(const pugi::xml_node &node, const char *name)
    {
        for (pugi::xml_node child : node.children())
        {
            if (NodeNameIs(child, name))
                return child;
            pugi::xml_node nested = FindFirstDescendant(child, name);
            if (nested)
                return nested;
        }
        return {};
    }

    // 读取 XML 属性并转为 double, 失败返回 false
    bool AttrDouble(const pugi::xml_node &node, const char *name, double *out)
    {
        pugi::xml_attribute attr = node.attribute(name);
        if (!attr)
            return false;
        try
        {
            *out = std::stod(attr.value());
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // 读取 XML 属性并转为 int64, 失败返回 false
    bool AttrInt64(const pugi::xml_node &node, const char *name, int64_t *out)
    {
        pugi::xml_attribute attr = node.attribute(name);
        if (!attr)
            return false;
        try
        {
            *out = std::stoll(attr.value());
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

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
            if (map_loaded_ && TryInstallRouteFromXosc(route_xosc_path_))
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
        // ========================================================================
        // 内部数据结构
        // ========================================================================

        // 路段索引: 由 Route waypoint 按 road_id 合并得来, 用于 road/lane 级 tracking
        struct RuntimeRouteSegment
        {
            int64_t road_id = 0;
            int lane_id = 0;
            double s_start = 0.0;
            double s_end = 0.0;
            double s_sign = 1.0; // +1 沿 s 正方向, -1 反方向
            double t = 0.0;      // 道路参考线横向偏移
            size_t wp_start_idx = 0;
            size_t wp_end_idx = 0;
        };

        // XOSC 轨迹顶点: 包含道路坐标和对应的世界坐标
        struct XoscRoutePoint
        {
            bool has_road = false;
            int64_t road_id = 0;
            int lane_id = 0;
            double s = 0.0;
            double t = 0.0;
            rsim_driver::WorldPoint world;
        };

        // ---- XOSC 解析 ----

        // 解析 XOSC RoadPosition 顶点: roadId/s/t → TrackToWorld 转世界坐标 → 填入 XoscRoutePoint
        bool ParseXoscRoadPosition(const pugi::xml_node &roadPos, XoscRoutePoint *out) const
        {
            int64_t road_id = 0;
            double s = 0.0;
            double t = 0.0;
            if (!AttrInt64(roadPos, "roadId", &road_id) ||
                !AttrDouble(roadPos, "s", &s))
            {
                return false;
            }
            AttrDouble(roadPos, "t", &t);

            rsim_driver::WorldPose pose = map_.TrackToWorld(road_id, s, t);
            if (!pose.valid)
                return false;

            out->has_road = true;
            out->road_id = road_id;
            out->s = s;
            out->t = t;
            out->lane_id = map_.TrackTToLane(road_id, s, t);
            out->world.x = pose.x;
            out->world.y = pose.y;
            return true;
        }

        // 解析 XOSC WorldPosition 顶点: 直接读取 x/y, 无需道路坐标转换
        bool ParseXoscWorldPosition(const pugi::xml_node &worldPos, XoscRoutePoint *out) const
        {
            double x = 0.0;
            double y = 0.0;
            if (!AttrDouble(worldPos, "x", &x) ||
                !AttrDouble(worldPos, "y", &y))
            {
                return false;
            }
            out->has_road = false;
            out->world.x = x;
            out->world.y = y;
            return true;
        }

        // 判断两个 XOSC 轨迹顶点是否重复 (世界坐标距离 < 1mm 且道路坐标一致)
        bool IsDuplicateXoscPoint(const XoscRoutePoint &a, const XoscRoutePoint &b) const
        {
            const double dx = a.world.x - b.world.x;
            const double dy = a.world.y - b.world.y;
            if (dx * dx + dy * dy > 1e-6)
                return false;
            if (a.has_road != b.has_road)
                return false;
            if (!a.has_road)
                return true;
            return a.road_id == b.road_id &&
                   std::fabs(a.s - b.s) < 1e-6 &&
                   std::fabs(a.t - b.t) < 1e-6;
        }

        // 解析 Polyline 下的 Vertex 列表, 去重后返回; 至少需要 4 个有效顶点
        bool ParseXoscPolyline(const pugi::xml_node &polyline,
                               std::vector<XoscRoutePoint> *out) const
        {
            std::vector<XoscRoutePoint> parsed;
            for (pugi::xml_node vertex : polyline.children("Vertex"))
            {
                pugi::xml_node position = vertex.child("Position");
                if (!position)
                    continue;

                XoscRoutePoint point;
                if (pugi::xml_node roadPos = position.child("RoadPosition"))
                {
                    if (!ParseXoscRoadPosition(roadPos, &point))
                        continue;
                }
                else if (pugi::xml_node worldPos = position.child("WorldPosition"))
                {
                    if (!ParseXoscWorldPosition(worldPos, &point))
                        continue;
                }
                else
                {
                    continue;
                }

                if (!parsed.empty() && IsDuplicateXoscPoint(parsed.back(), point))
                    continue;
                parsed.push_back(point);
            }

            if (parsed.size() < 4)
                return false;

            *out = std::move(parsed);
            return true;
        }

        // 递归搜索 FollowTrajectoryAction 节点, 找到后提取其 Polyline 顶点
        bool TryParseFollowTrajectoryActions(const pugi::xml_node &node,
                                             std::vector<XoscRoutePoint> *out) const
        {
            if (NodeNameIs(node, "FollowTrajectoryAction"))
            {
                pugi::xml_node polyline = FindFirstDescendant(node, "Polyline");
                if (polyline && ParseXoscPolyline(polyline, out))
                    return true;
            }

            for (pugi::xml_node child : node.children())
            {
                if (TryParseFollowTrajectoryActions(child, out))
                    return true;
            }
            return false;
        }

        // 从 XOSC 中提取指定实体的 FollowTrajectoryAction 轨迹: 只匹配 Private[@entityRef=entityName]
        bool TryExtractXoscTrajectory(const pugi::xml_node &node,
                                      const std::string &entityName,
                                      std::vector<XoscRoutePoint> *out) const
        {
            if (NodeNameIs(node, "Private"))
            {
                pugi::xml_attribute entityRef = node.attribute("entityRef");
                if (entityRef && entityName == entityRef.value() &&
                    TryParseFollowTrajectoryActions(node, out))
                {
                    return true;
                }
            }

            for (pugi::xml_node child : node.children())
            {
                if (TryExtractXoscTrajectory(child, entityName, out))
                    return true;
            }
            return false;
        }

        // 将 XOSC 顶点按 road_id 合并为路段索引 route_segments_, 用于 road/lane 级追踪
        void BuildRouteSegmentsFromXoscPoints(const std::vector<XoscRoutePoint> &points)
        {
            route_segments_.clear();
            std::size_t i = 0;
            while (i < points.size())
            {
                if (!points[i].has_road)
                {
                    ++i;
                    continue;
                }

                const int64_t road_id = points[i].road_id;
                std::size_t j = i;
                int first_nonzero_lane = 0;
                while (j < points.size() && points[j].has_road &&
                       points[j].road_id == road_id)
                {
                    if (first_nonzero_lane == 0 && points[j].lane_id != 0)
                        first_nonzero_lane = points[j].lane_id;
                    ++j;
                }

                RuntimeRouteSegment r;
                r.road_id = road_id;
                r.s_start = points[i].s;
                r.s_end = points[j - 1].s;
                r.s_sign = (r.s_end >= r.s_start) ? +1.0 : -1.0;
                r.t = points[i].t;
                r.lane_id = (first_nonzero_lane != 0)
                                ? first_nonzero_lane
                                : map_.TrackTToLane(r.road_id, r.s_start, r.t);
                r.wp_start_idx = i;
                r.wp_end_idx = j;
                route_segments_.push_back(r);
                i = j;
            }
        }

        // 打印全局路径摘要到 stderr: 原始顶点数 / 路段数 / 世界坐标点数 / 总弦长
        void DumpXoscTrajectoryRoute(const std::vector<XoscRoutePoint> &points,
                                     const std::string &xoscPath) const
        {
            /*std::fprintf(stderr,
                         "\n"
                         "[RSimDriver] ==============================================================\n"
                         "[RSimDriver] XOSC trajectory route installed\n"
                         "[RSimDriver]   xoscPath          = %s\n"
                         "[RSimDriver]   rawTrajectoryPts  = %zu\n"
                         "[RSimDriver]   routeSegments     = %zu\n"
                         "[RSimDriver]   worldPoints       = %zu\n",
                         xoscPath.c_str(), points.size(),
                         route_segments_.size(), global_path_world_points_.size());

            std::fprintf(stderr,
                         "[RSimDriver]   世界坐标点总弦长: %.2f m\n"
                         "[RSimDriver] ==============================================================\n\n",
                         ComputeWorldPointsChordLength());
                         */
        }

        // 主入口: 加载 XOSC 文件 → 提取轨迹 → 建路段索引 → 加密世界坐标点 → 写入成员变量
        bool TryInstallRouteFromXosc(const std::string &xoscPath)
        {
            if (xoscPath.empty())
                return false;

            pugi::xml_document doc;
            const pugi::xml_parse_result load = doc.load_file(xoscPath.c_str());
            if (!load)
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: routeXoscPath 解析失败 '%s': %s\n",
                             xoscPath.c_str(), load.description());
                return false;
            }

            std::vector<XoscRoutePoint> points;
            if (!TryExtractXoscTrajectory(doc, entity_name_, &points))
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: routeXoscPath 中未找到 entityRef=\"%s\" 的有效 FollowTrajectoryAction\n",
                             entity_name_.c_str());
                return false;
            }

            BuildRouteSegmentsFromXoscPoints(points);

            global_path_world_points_.clear();
            global_path_world_points_.reserve(points.size());
            for (const auto &point : points)
                global_path_world_points_.push_back(point.world);

            if (route_segments_.empty() || global_path_world_points_.size() < 4)
            {
                route_segments_.clear();
                global_path_world_points_.clear();
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: XOSC trajectory route 点数/路段不足\n");
                return false;
            }

            closest_global_path_idx_ = 0;
            //DumpXoscTrajectoryRoute(points, xoscPath);
            WriteGlobalPathCsv();
            return true;
        }

        // 将 global_path_world_points_ 导出为 CSV
        void WriteGlobalPathCsv() const
        {
            if (route_csv_path_.empty() || global_path_world_points_.empty())
                return;

            std::FILE *fp = std::fopen(route_csv_path_.c_str(), "w");
            if (fp == nullptr)
            {
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: 无法写入全局路径 CSV: %s\n",
                             route_csv_path_.c_str());
                return;
            }

            std::fprintf(fp, "index,arc_s,delta_s,x,y\n");
            double arcS = 0.0;
            for (size_t i = 0; i < global_path_world_points_.size(); ++i)
            {
                double ds = 0.0;
                if (i > 0)
                {
                    const double dx = global_path_world_points_[i].x - global_path_world_points_[i - 1].x;
                    const double dy = global_path_world_points_[i].y - global_path_world_points_[i - 1].y;
                    ds = std::sqrt(dx * dx + dy * dy);
                    arcS += ds;
                }

                const auto &pt = global_path_world_points_[i];
                std::fprintf(fp, "%zu,%.9f,%.9f,%.9f,%.9f\n",
                             i, arcS, ds, pt.x, pt.y);
            }
            std::fclose(fp);

            std::fprintf(stderr,
                         "[RSimDriver]###INIT运行 global_path CSV written: %s (rows=%zu, chord=%.2f m)\n",
                         route_csv_path_.c_str(), global_path_world_points_.size(),
                         ComputeWorldPointsChordLength());
        }

        // 计算 global_path_world_points_ 的总弦长 (相邻点欧氏距离累加)
        double ComputeWorldPointsChordLength() const
        {
            if (global_path_world_points_.size() < 2)
                return 0.0;
            double total = 0.0;
            for (size_t i = 1; i < global_path_world_points_.size(); ++i)
            {
                double dx = global_path_world_points_[i].x - global_path_world_points_[i - 1].x;
                double dy = global_path_world_points_[i].y - global_path_world_points_[i - 1].y;
                total += std::sqrt(dx * dx + dy * dy);
            }
            return total;
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
                             "[RSimDriver]###初始运行 LatchInitialState: ego scene=(%.2f, %.2f) 主车 → "
                             "绑定到 route start seg[0] road=%lld lane=%d s=%.2f（路段）, "
                             "worldPt[0] (%.2f, %.2f)\n",
                             ego->x, ego->y,
                             static_cast<long long>(current_road_),
                             current_lane_, current_s_,
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
                             reference_line_->points.size()
                             );
                reference_line_ready_reported_ = true;
            }
            //std::printf("globalMatchIdx=%zu \n", reference_line_generator_.lastMatchPointIndex());
        }

        // ========================================================================
        // 成员变量
        // ========================================================================
        std::vector<int32_t> controlled_ids_;
        rsim_driver::MapHelper map_;

        // ---- 全局路由 — 路段索引 (用于 road/lane tracking) ----
        std::vector<RuntimeRouteSegment> route_segments_;

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
