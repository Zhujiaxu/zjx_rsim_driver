/*
 * ============================================================================
 * RSimDriverPlugin — 基于全局路径的局部轨迹规划插件
 * ============================================================================
 *
 * ---- 整体数据流 ----
 *
 *   xosc 场景文件
 *     ├── Init → FollowTrajectoryAction (ego 的初始轨迹，RoadPosition 序列)
 *     └── Story → SetEgoRoute (全局路由的起止航路点)
 *            │
 *            ▼
 *   SceneRunner (仿真平台)
 *     ├── 解析 SetEgoRoute，通过路网拓扑展开成完整 Route
 *     └── 提供 RPC API: GetRoute(entityName) → rsim::rpc::Route
 *            │
 *            ▼
 *   RSimDriverPlugin::Init()
 *     └── StartRouteFetch() — 启动后台线程，异步调用 GetRoute("ego")
 *            │
 *            ▼
 *   RSimDriverPlugin::Step() (每帧)
 *     ├── PollFetchedRoute()
 *     │     ├── Route 到达后:
 *     │     │   1. BuildRouteSegmentsFromRoute() → route_segments_ (路段索引, 用于tracking)
 *     │     │   2. ExtractAndDensifyWorldPoints() → global_path_world_points_ (世界坐标, 用于参考线)
 *     │     │
 *     │     ├── BuildPlannerFrame()
 *     │     │   ├── 有全局路由 → 从 global_path_world_points_ 中截取 ego 前方子集
 *     │     │   │               → ProvideFromWorldPoints()  直接用世界坐标建参考线
 *     │     │   └── 无全局路由 → Provide()  单路段 fallback
 *     │     │
 *     │     ├── PlanFrame() → IPlanner::Plan()
 *     │     └── ExecutePlan() + BuildActorUpdate()
 *
 * ---- 关键设计决策: 参考线直接从世界坐标构建 ----
 *
 *   旧方案: Route waypoint (road_id, s, t) → TrackToWorld() → 道路参考线映射 → (x,y)
 *   新方案: Route waypoint (x_val, y_val, heading) → 直接使用 → 参考线
 *
 *   为什么?
 *   - Route 的每个 WayPoint 已经包含了准确的世界坐标 (x_val, y_val, heading),
 *     这是 SceneRunner 通过路网拓扑计算出来的, 精度等同于地图。
 *   - 跳过 TrackToWorld() 减少了中间环节, 避免了道路参考线的映射误差。
 *   - 参考线从 Route 世界坐标中"截取并平滑", 符合用户的架构意图。
 *
 * ---- 全局路径 (Global Route) 数据结构 ----
 *
 *   rsim::rpc::Route
 *     └── vector<WayPoint>
 *           每个 WayPoint 包含:
 *             track_id : 道路 ID
 *             lane_id  : 车道 ID
 *             s        : 道路 Frenet 纵向位置
 *             t        : 相对道路 reference line 的横向偏移
 *             x_val    : 世界坐标 X  ← 用于参考线构建
 *             y_val    : 世界坐标 Y  ← 用于参考线构建
 *             heading  : 世界航向角   ← 用于参考线构建
 *
 *   插件内部两份数据:
 *     1. route_segments_[]          — 路段索引 (road_id, lane_id, s区间)
 *                                    用于 road/lane tracking, perception, actor update
 *     2. global_path_world_points_[] — 世界坐标点序列 (x, y, hdg)
 *                                    用于参考线构建 (ProvideFromWorldPoints)
 *
 * ---- 验证全局路径的方法 ----
 *
 *   运行仿真后，观察 stderr 输出:
 *     [RSimDriver] ====== Global Route Received ======
 *     ... 打印所有 waypoint 和 world point 信息 ...
 *     [RSimDriver] =====================================
 *     如果看到以上输出，说明全局路径获取成功。
 *     如果看到 "WARNING: 全局路由获取失败"，说明 RPC 调用失败。
 *
 * ---- 配置参数 (xosc properties / plugin.yaml) ----
 *
 *   xodrPath          : OpenDRIVE 地图绝对路径 (必需)
 *   setSpeed          : 期望巡航速度 m/s (默认 10.0)
 *   maxAccel          : 最大加速度 m/s² (默认 3.0)
 *   maxDecel          : 最大减速度 m/s² (默认 4.0)
 *   plannerType       : "sampling" 或 "em" (默认 "sampling")
 *   planningHorizonSec: 规划时域 秒 (默认 5.0)
 *   enableDebugLog    : 是否打印调试日志 (默认 false)
 *   sceneRunnerHost   : SceneRunner RPC 主机 (默认 127.0.0.1)
 *   sceneRunnerPort   : SceneRunner RPC 端口 (默认 9110)
 *   entityName        : 要获取 Route 的实体名 (默认 "ego")
 * ============================================================================
 */

#include "rsim/worldsim_plugin/PluginInterface.hpp"
#include "rsim/client/SceneRunnerClient.h"

#include "pugixml.hpp"

#include "DriverTypes.hpp"
#include "EMPlanner.hpp"
#include "IPlanner.hpp"
#include "MapHelper.hpp"
#include "PerceptionHelper.hpp"
#include "ReferenceLine.hpp"
#include "ReferenceLineProvider.hpp"
#include "SamplingPlanner.hpp"
#include "TrafficRuleHandler.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using rsim_plugin::ActorState;
using rsim_plugin::ActorUpdate;
using rsim_plugin::IPluginController;
using rsim_plugin::TickContext;

namespace
{

bool NodeNameIs(const pugi::xml_node& node, const char* name)
{
    return std::strcmp(node.name(), name) == 0;
}

pugi::xml_node FindFirstDescendant(const pugi::xml_node& node, const char* name)
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

bool AttrDouble(const pugi::xml_node& node, const char* name, double* out)
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

bool AttrInt64(const pugi::xml_node& node, const char* name, int64_t* out)
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
        if (route_fetcher_.joinable())
            route_fetcher_.join();
    }

    // ========================================================================
    // Init() — 插件初始化
    // ========================================================================
    void Init(const std::map<std::string, std::string>& properties,
              const std::vector<int32_t>& controlled_actor_ids) override
    {
        controlled_ids_ = controlled_actor_ids;

        auto getStr = [&](const char* key, const std::string& def) {
            auto it = properties.find(key);
            return (it != properties.end()) ? it->second : def;
        };
        auto getDouble = [&](const char* key, double def) -> double {
            auto it = properties.find(key);
            if (it == properties.end()) return def;
            try { return std::stod(it->second); } catch (...) { return def; }
        };

        // ---- 基本配置 ----
        std::string xodr_path = getStr("xodrPath", "");
        route_xosc_path_ = getStr("routeXoscPath", "");
        planner_type_ = getStr("plannerType", "sampling");
        set_speed_  = getDouble("setSpeed", 10.0);
        max_accel_  = getDouble("maxAccel", 3.0);
        max_decel_  = getDouble("maxDecel", 4.0);
        lateral_dist_ = getDouble("lateralDist", 2.0);
        planning_horizon_sec_ = getDouble("planningHorizonSec", 5.0);
        enable_debug_log_ = getStr("enableDebugLog", "false") == "true";

        // ---- SceneRunner RPC 连接参数 ----
        sr_host_     = getStr("sceneRunnerHost", "127.0.0.1");
        sr_port_     = static_cast<uint16_t>(
            static_cast<int>(getDouble("sceneRunnerPort", 9110.0)));
        entity_name_ = getStr("entityName", "ego");

        // ---- 构造规划器 ----
        if (planner_type_ == "em")
        {
            auto* em = new rsim_driver::EMPlanner();
            em->params.maxAccel = max_accel_;
            em->params.maxDecel = max_decel_;
            em->params.maxSpeed = std::max(set_speed_ + 5.0, set_speed_ * 1.5);
            planner_.reset(em);
        }
        else
        {
            auto* sp = new rsim_driver::SamplingPlanner();
            sp->params.maxAccel = max_accel_;
            sp->params.maxDecel = max_decel_;
            sp->params.comfortDecel = std::max(0.5, max_decel_ * 0.5);
            sp->params.maxSpeed = std::max(set_speed_ + 5.0, set_speed_ * 1.5);
            planner_.reset(sp);
        }

        // ---- 加载 OpenDRIVE 地图 (仍需地图用于 perception/lane lookup) ----
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

        // ---- 优先从 runtime XOSC 读取 ego FollowTrajectoryAction ----
        // 06 场景里 SetEgoRoute 只有起终点, 完整路径在 Init/Private 的
        // FollowTrajectoryAction 里。XOSC 读取失败时再用 GetRoute 兜底。
        bool route_fetch_started = false;
        if (map_loaded_ && TryInstallRouteFromXosc(route_xosc_path_))
        {
            route_fetch_done_.store(true, std::memory_order_release);
        }
        else
        {
            // 异步原因: Init() 和 Step() 都在 SceneRunner tick-sync 路径上,
            // 同步 RPC 调用会因单线程 worker 被自身阻塞而 deadlock。
            StartRouteFetch();
            route_fetch_started = true;
        }

        std::fprintf(stderr,
                     "[RSimDriver] ========================================================\n"
                     "[RSimDriver] Init done:\n"
                     "[RSimDriver]   plannerType       = %s\n"
                     "[RSimDriver]   setSpeed          = %.2f m/s\n"
                     "[RSimDriver]   maxAccel          = %.2f m/s2\n"
                     "[RSimDriver]   maxDecel          = %.2f m/s2\n"
                     "[RSimDriver]   planningHorizon   = %.2f s\n"
                     "[RSimDriver]   xodrPath          = %s\n"
                     "[RSimDriver]   routeXoscPath     = %s\n"
                     "[RSimDriver]   mapLoaded          = %s\n"
                     "[RSimDriver]   sceneRunnerHost    = %s\n"
                     "[RSimDriver]   sceneRunnerPort    = %d\n"
                     "[RSimDriver]   entityName         = %s\n"
                     "[RSimDriver]   controlledActors   = %zu\n"
                     "[RSimDriver]   routeSource         = %s\n"
                     "[RSimDriver]   routeFetch          = %s\n"
                     "[RSimDriver] ========================================================\n",
                     planner_type_.c_str(),
                     set_speed_,
                     max_accel_,
                     max_decel_,
                     planning_horizon_sec_,
                     xodr_path.c_str(),
                     route_xosc_path_.empty() ? "(none)" : route_xosc_path_.c_str(),
                     map_loaded_ ? "ok" : "FAILED",
                     sr_host_.c_str(),
                     static_cast<int>(sr_port_),
                     entity_name_.c_str(),
                     controlled_ids_.size(),
                     route_source_.c_str(),
                     route_fetch_started ? "started (waiting for RPC...)"
                                         : "skipped (XOSC trajectory route installed)");
    }

    // ========================================================================
    // Step() — 每帧主循环
    // ========================================================================
    std::vector<ActorUpdate> Step(const TickContext& ctx) override
    {
        std::vector<ActorUpdate> updates;
        if (!map_loaded_ || controlled_ids_.empty() || !planner_)
            return updates;

        const ActorState* ego = FindControlledActor(ctx);
        if (ego == nullptr)
            return updates;

        PollFetchedRoute();
        LatchInitialState(*ego);

        PlannerFrame frame = BuildPlannerFrame(ctx, *ego);
        const PlannerExecution plan = PlanFrame(ctx, frame);
        ExecutePlan(plan, frame, ctx.time_step);

        updates.push_back(BuildActorUpdate(ego->id));
        return updates;
    }

private:
    // ========================================================================
    // 内部数据结构
    // ========================================================================

    struct PlannerFrame
    {
        ActorState egoView {};
        rsim_driver::ScanResult scan;
        rsim_driver::Obstacle planObstacles[rsim_driver::ScanResult::MAX_OBSTACLES];
        rsim_driver::FrenetState currentState;
        rsim_driver::ReferenceLineInfo reference;
        std::unique_ptr<rsim_driver::ReferenceLine> refLine; // 规划参考线
        double planSSign = 1.0;
        int numPlanObstacles = 0;
        bool usingGlobalRoute = false;
    };

    struct PlannerExecution
    {
        bool valid = false;
        rsim_driver::FrenetTrajectory frenet;
        rsim_driver::PlannedTrajectory trajectory;
    };

    // 路段索引: 由 Route waypoint 按 road_id 合并得来, 用于 road/lane 级 tracking
    // (LatchInitialState, AdvanceRouterIfSegmentExhausted, BuildActorUpdate, 地图查询)
    struct RuntimeRouteSegment
    {
        int64_t road_id = 0;
        int     lane_id = 0;
        double  s_start = 0.0;
        double  s_end   = 0.0;
        double  s_sign  = 1.0;  // +1 沿 s 正方向, -1 反方向
        double  t       = 0.0;  // 道路参考线横向偏移
        // 该 segment 对应 global_path_world_points_ 中的索引范围 [wp_start_idx, wp_end_idx)
        size_t  wp_start_idx = 0;
        size_t  wp_end_idx   = 0;
    };

    struct XoscRoutePoint
    {
        bool    has_road = false;
        int64_t road_id  = 0;
        int     lane_id  = 0;
        double  s        = 0.0;
        double  t        = 0.0;
        rsim_driver::WorldPoint world;
    };

    // ========================================================================
    // 全局路由获取 — 异步 RPC
    // ========================================================================

    // 启动后台线程, 通过 SceneRunner RPC 获取 ego 的全局路由。
    // GetRoute() 返回的 Route 源自 xosc 中的 SetEgoRoute 自定义命令,
    // SceneRunner 将起止航路点通过路网拓扑展开为完整的 WayPoint 序列。
    void StartRouteFetch()
    {
        route_fetcher_ = std::thread([this]() {
            try
            {
                std::fprintf(stderr,
                             "[RSimDriver] 后台线程: 连接 SceneRunner %s:%u ...\n",
                             sr_host_.c_str(), static_cast<unsigned>(sr_port_));

                rsim::client::SceneRunnerClient client(sr_host_, sr_port_);

                std::fprintf(stderr,
                             "[RSimDriver] 后台线程: 已连接, 正在 GetRoute(\"%s\") ...\n",
                             entity_name_.c_str());

                // 【核心 RPC 调用】返回的 Route 每个 WayPoint 包含:
                //   track_id, lane_id, s, t  (道路网络定位)
                //   x_val, y_val, z_val, heading  (世界坐标 — 直接用于参考线!)
                rsim::rpc::Route route = client.GetRoute(entity_name_);

                const auto& wps = route.GetWaypoints();
                std::fprintf(stderr,
                             "[RSimDriver] 后台线程: GetRoute 成功! 共 %zu 个 WayPoint\n",
                             wps.size());

                {
                    std::lock_guard<std::mutex> g(pending_route_mtx_);
                    pending_route_ = std::move(route);
                    pending_route_ok_ = true;
                }
                route_fetch_done_.store(true, std::memory_order_release);
            }
            catch (const std::exception& e)
            {
                std::fprintf(stderr,
                             "[RSimDriver] 后台线程: GetRoute(\"%s\") 失败! %s:%u → %s\n"
                             "[RSimDriver]   将使用 fallback 单路段 lane-following 模式\n",
                             entity_name_.c_str(), sr_host_.c_str(),
                             static_cast<unsigned>(sr_port_), e.what());
                route_fetch_done_.store(true, std::memory_order_release);
            }
        });
    }

    // 每帧在主线程中轮询: 后台线程是否已完成全局路由获取。
    // 完成后做两件事:
    //   1. BuildRouteSegmentsFromRoute() → route_segments_ (路段索引, 用于 tracking)
    //   2. ExtractAndDensifyWorldPoints() → global_path_world_points_ (世界坐标, 用于参考线)
    void PollFetchedRoute()
    {
        if (route_installed_) return;
        if (!route_fetch_done_.load(std::memory_order_acquire)) return;

        bool ok = false;
        rsim::rpc::Route route;
        {
            std::lock_guard<std::mutex> g(pending_route_mtx_);
            ok = pending_route_ok_;
            if (ok) route = std::move(pending_route_);
        }

        if (ok)
        {
            const auto& wps = route.GetWaypoints();
            if (wps.size() < static_cast<size_t>(refLineProvider_.config.minPoints))
            {
                route_segments_.clear();
                global_path_world_points_.clear();
                route_source_ = "fallback-lane";
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: GetRoute(\"%s\") 仅返回 %zu 个 WayPoint, "
                             "不足以构建全局参考线; 使用单路段 fallback, 避免起终点直线穿图\n",
                             entity_name_.c_str(), wps.size());
                route_installed_ = true;
                if (route_fetcher_.joinable())
                    route_fetcher_.join();
                return;
            }

            // 步骤1: 构建路段索引 (road_id / lane_id / s 区间)
            BuildRouteSegmentsFromRoute(route);

            // 步骤2: 从 Route waypoint 提取世界坐标, 插值加密到 ~1m
            ExtractAndDensifyWorldPoints(route);

            // 验证用: 打印完整路径信息
            DumpGlobalRoute(route);

            if (global_path_world_points_.size() <
                static_cast<size_t>(refLineProvider_.config.minPoints))
            {
                route_segments_.clear();
                global_path_world_points_.clear();
                route_source_ = "fallback-lane";
                std::fprintf(stderr,
                             "[RSimDriver] WARNING: GetRoute worldPoints 不足, 使用单路段 fallback\n");
            }
            else
            {
                route_source_ = "sceneRunner-GetRoute";
                std::fprintf(stderr,
                             "[RSimDriver] 全局路由已安装: routeSegments=%zu  worldPoints=%zu\n"
                             "[RSimDriver]   参考线将从世界坐标直接构建 (ProvideFromWorldPoints)\n",
                             route_segments_.size(), global_path_world_points_.size());
            }
        }
        else
        {
            route_source_ = "fallback-lane";
            std::fprintf(stderr,
                         "[RSimDriver] WARNING: 全局路由获取失败, 使用单路段 fallback\n");
        }
        route_installed_ = true;

        if (route_fetcher_.joinable())
            route_fetcher_.join();
    }

    // ========================================================================
    // 从 Route waypoint 构建路段索引
    // ========================================================================
    // 连续相同 track_id 的 waypoint 合并为一个 segment。
    // route_segments_ 只用于 road/lane 级 tracking, 不参与参考线构建。
    void BuildRouteSegmentsFromRoute(const rsim::rpc::Route& route)
    {
        route_segments_.clear();
        const auto& wps = route.GetWaypoints();
        if (wps.empty()) return;

        std::size_t i = 0;
        while (i < wps.size())
        {
            const int64_t road_id = wps[i].track_id;

            std::size_t j = i;
            int first_nonzero_lane = 0;
            while (j < wps.size() && wps[j].track_id == road_id)
            {
                if (first_nonzero_lane == 0 && wps[j].lane_id != 0)
                    first_nonzero_lane = wps[j].lane_id;
                ++j;
            }

            RuntimeRouteSegment r;
            r.road_id     = road_id;
            r.s_start     = wps[i].s;
            r.s_end       = wps[j - 1].s;
            r.s_sign      = (r.s_end >= r.s_start) ? +1.0 : -1.0;
            r.t           = wps[i].t;
            r.lane_id     = (first_nonzero_lane != 0)
                                ? first_nonzero_lane
                                : map_.TrackTToLane(r.road_id, r.s_start, r.t);
            r.wp_start_idx = i;
            r.wp_end_idx   = j;
            route_segments_.push_back(r);
            i = j;
        }
    }

    // ========================================================================
    // 从 Route waypoint 提取世界坐标, 并线性插值加密到 ~1m 间距
    // ========================================================================
    // 直接使用 WayPoint 中的 (x_val, y_val, heading), 不经过道路参考线映射。
    // 在相邻 waypoint 之间做线性插值, 使最终点间距 ≤ 1.0m,
    // 保证参考线平滑质量 (太稀疏的点会导致曲率计算不准)。
    void ExtractAndDensifyWorldPoints(const rsim::rpc::Route& route)
    {
        const auto& wps = route.GetWaypoints();
        if (wps.empty()) return;

        std::vector<rsim_driver::WorldPoint> raw;
        raw.reserve(wps.size());

        for (size_t i = 0; i < wps.size(); ++i)
        {
            rsim_driver::WorldPoint pt;
            pt.x   = wps[i].x_val;
            pt.y   = wps[i].y_val;
            pt.hdg = wps[i].heading;
            raw.push_back(pt);
        }

        DensifyWorldPoints(raw);
    }

    void DensifyWorldPoints(const std::vector<rsim_driver::WorldPoint>& raw)
    {
        global_path_world_points_.clear();
        if (raw.empty()) return;

        global_path_world_points_.reserve(raw.size() * 2);
        for (size_t i = 0; i < raw.size(); ++i)
        {
            global_path_world_points_.push_back(raw[i]);
            if (i + 1 >= raw.size()) break;

            const double dx = raw[i + 1].x - raw[i].x;
            const double dy = raw[i + 1].y - raw[i].y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 1.5)
            {
                const int numInsert = static_cast<int>(std::ceil(dist / 1.0)) - 1;
                for (int k = 1; k <= numInsert; ++k)
                {
                    const double ratio = static_cast<double>(k) / static_cast<double>(numInsert + 1);

                    // 线性插值 x, y
                    rsim_driver::WorldPoint interp;
                    interp.x = raw[i].x + ratio * dx;
                    interp.y = raw[i].y + ratio * dy;

                    // 航向角插值 (处理角度环绕)
                    double dh = raw[i + 1].hdg - raw[i].hdg;
                    while (dh > M_PI)  dh -= 2.0 * M_PI;
                    while (dh < -M_PI) dh += 2.0 * M_PI;
                    interp.hdg = raw[i].hdg + ratio * dh;

                    global_path_world_points_.push_back(interp);
                }
            }
        }
    }

    bool ParseXoscRoadPosition(const pugi::xml_node& roadPos, XoscRoutePoint* out) const
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
        out->world.hdg = pose.h;

        pugi::xml_node orientation = roadPos.child("Orientation");
        double h = 0.0;
        if (orientation && AttrDouble(orientation, "h", &h))
            out->world.hdg = h;
        return true;
    }

    bool ParseXoscWorldPosition(const pugi::xml_node& worldPos, XoscRoutePoint* out) const
    {
        double x = 0.0;
        double y = 0.0;
        double h = 0.0;
        if (!AttrDouble(worldPos, "x", &x) ||
            !AttrDouble(worldPos, "y", &y))
        {
            return false;
        }
        AttrDouble(worldPos, "h", &h);
        out->has_road = false;
        out->world.x = x;
        out->world.y = y;
        out->world.hdg = h;
        return true;
    }

    bool IsDuplicateXoscPoint(const XoscRoutePoint& a, const XoscRoutePoint& b) const
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

    bool ParseXoscPolyline(const pugi::xml_node& polyline,
                           std::vector<XoscRoutePoint>* out) const
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

        if (parsed.size() < static_cast<size_t>(refLineProvider_.config.minPoints))
            return false;

        *out = std::move(parsed);
        return true;
    }

    bool TryParseFollowTrajectoryActions(const pugi::xml_node& node,
                                         std::vector<XoscRoutePoint>* out) const
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

    bool TryExtractXoscTrajectory(const pugi::xml_node& node,
                                  const std::string& entityName,
                                  std::vector<XoscRoutePoint>* out) const
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

    void BuildRouteSegmentsFromXoscPoints(const std::vector<XoscRoutePoint>& points)
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

    void DumpXoscTrajectoryRoute(const std::vector<XoscRoutePoint>& points,
                                 const std::string& xoscPath) const
    {
        std::fprintf(stderr,
                     "\n"
                     "[RSimDriver] ==============================================================\n"
                     "[RSimDriver] XOSC trajectory route installed\n"
                     "[RSimDriver]   xoscPath          = %s\n"
                     "[RSimDriver]   rawTrajectoryPts  = %zu\n"
                     "[RSimDriver]   routeSegments     = %zu\n"
                     "[RSimDriver]   worldPoints       = %zu\n",
                     xoscPath.c_str(), points.size(),
                     route_segments_.size(), global_path_world_points_.size());

        const size_t maxShow = 12;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (i == maxShow && points.size() > maxShow + 5)
            {
                std::fprintf(stderr,
                             "[RSimDriver]   ... 省略 %zu 个中间 trajectory point ...\n",
                             points.size() - maxShow - 5);
                i = points.size() - 5;
                if (i <= maxShow) break;
            }
            const auto& pt = points[i];
            std::fprintf(stderr,
                         "[RSimDriver]   xoscPt[%3zu]: road=%5lld lane=%3d "
                         "s=%10.4f t=%8.4f x=%12.4f y=%12.4f h=%8.4f\n",
                         i, static_cast<long long>(pt.road_id), pt.lane_id,
                         pt.s, pt.t, pt.world.x, pt.world.y, pt.world.hdg);
        }

        std::fprintf(stderr,
                     "[RSimDriver]   世界坐标点总弦长: %.2f m\n"
                     "[RSimDriver] ==============================================================\n\n",
                     ComputeWorldPointsChordLength());
    }

    bool TryInstallRouteFromXosc(const std::string& xoscPath)
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

        std::vector<rsim_driver::WorldPoint> raw;
        raw.reserve(points.size());
        for (const auto& point : points)
            raw.push_back(point.world);
        DensifyWorldPoints(raw);

        if (route_segments_.empty() ||
            global_path_world_points_.size() <
                static_cast<size_t>(refLineProvider_.config.minPoints))
        {
            route_segments_.clear();
            global_path_world_points_.clear();
            std::fprintf(stderr,
                         "[RSimDriver] WARNING: XOSC trajectory route 点数/路段不足, 将尝试 GetRoute 兜底\n");
            return false;
        }

        current_segment_idx_ = 0;
        closest_global_path_idx_ = 0;
        route_source_ = "xosc-FollowTrajectory";
        route_installed_ = true;
        DumpXoscTrajectoryRoute(points, xoscPath);
        return true;
    }

    // ========================================================================
    // 在全局路径世界坐标中查找距离给定位置最近的点的索引
    // ========================================================================
    size_t FindClosestGlobalPathIndex(double x, double y, size_t searchFrom = 0) const
    {
        if (global_path_world_points_.empty()) return 0;

        double bestDist = std::numeric_limits<double>::max();
        size_t bestIdx = searchFrom;

        // 从 searchFrom 开始搜索, 假设 ego 在全局路径上是单调前进的
        for (size_t i = searchFrom; i < global_path_world_points_.size(); ++i)
        {
            const double dx = global_path_world_points_[i].x - x;
            const double dy = global_path_world_points_[i].y - y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < bestDist)
            {
                bestDist = d2;
                bestIdx = i;
            }
            // 如果距离开始变大且已经超过50m, 说明已走过最近点, 停止搜索
            if (d2 > bestDist * 4.0 && d2 > 2500.0) // 50m²
                break;
        }
        return bestIdx;
    }

    // ========================================================================
    // DumpGlobalRoute() — 打印全局路由详情, 用于验证
    // ========================================================================
    void DumpGlobalRoute(const rsim::rpc::Route& route)
    {
        const auto& wps = route.GetWaypoints();

        std::fprintf(stderr,
                     "\n"
                     "[RSimDriver] ==============================================================\n"
                     "[RSimDriver] ====== 全局路由已获取 (Global Route Received) ================\n"
                     "[RSimDriver] ==============================================================\n"
                     "[RSimDriver] 原始 WayPoint 数量: %zu\n"
                     "[RSimDriver] 路段 (Segment) 数量: %zu\n"
                     "[RSimDriver] 世界坐标点数量 (插值加密后): %zu\n",
                     wps.size(), route_segments_.size(), global_path_world_points_.size());

        // 打印每个原始 WayPoint
        std::fprintf(stderr,
                     "[RSimDriver] ---- 原始 WayPoint 列表 (前20个 + 后5个) ----\n");
        const size_t maxShow = 20;
        for (size_t i = 0; i < wps.size(); ++i)
        {
            if (i == maxShow && wps.size() > maxShow + 5)
            {
                std::fprintf(stderr,
                             "[RSimDriver]   ... 省略 %zu 个中间 waypoint ...\n",
                             wps.size() - maxShow - 5);
                i = wps.size() - 5;
                if (i <= maxShow) break;
            }
            const auto& wp = wps[i];
            std::fprintf(stderr,
                         "[RSimDriver]   wp[%3zu]: road=%5lld  lane=%3d  "
                         "s=%10.4f  t=%8.4f  x=%12.4f  y=%12.4f  h=%8.4f\n",
                         i,
                         static_cast<long long>(wp.track_id), wp.lane_id,
                         wp.s, wp.t, wp.x_val, wp.y_val, wp.heading);
        }

        // 打印路段索引
        std::fprintf(stderr,
                     "[RSimDriver] ---- 路段索引 (用于 road/lane tracking) ----\n");
        double totalLen = 0.0;
        for (size_t i = 0; i < route_segments_.size(); ++i)
        {
            const auto& seg = route_segments_[i];
            double segLen = std::fabs(seg.s_end - seg.s_start);
            totalLen += segLen;
            std::fprintf(stderr,
                         "[RSimDriver]   seg[%2zu]: road=%5lld  lane=%3d  "
                         "s=[%10.4f→%10.4f]  len=%8.2f  sign=%+2.0f  "
                         "wpRange=[%zu, %zu)\n",
                         i, static_cast<long long>(seg.road_id), seg.lane_id,
                         seg.s_start, seg.s_end, segLen, seg.s_sign,
                         seg.wp_start_idx, seg.wp_end_idx);
        }
        std::fprintf(stderr,
                     "[RSimDriver]   全局路由道路总里程: %.2f m\n"
                     "[RSimDriver]   世界坐标点总弦长: %.2f m\n",
                     totalLen, ComputeWorldPointsChordLength());
        std::fprintf(stderr,
                     "[RSimDriver] ==============================================================\n\n");
    }

    // 计算 global_path_world_points_ 的总弦长
    double ComputeWorldPointsChordLength() const
    {
        if (global_path_world_points_.size() < 2) return 0.0;
        double total = 0.0;
        for (size_t i = 1; i < global_path_world_points_.size(); ++i)
        {
            double dx = global_path_world_points_[i].x - global_path_world_points_[i - 1].x;
            double dy = global_path_world_points_[i].y - global_path_world_points_[i - 1].y;
            total += std::sqrt(dx * dx + dy * dy);
        }
        return total;
    }

    // ========================================================================
    // FindControlledActor
    // ========================================================================
    const ActorState* FindControlledActor(const TickContext& ctx) const
    {
        for (const auto& a : ctx.actors)
            if (a.id == controlled_ids_.front())
                return &a;
        return nullptr;
    }

    // ========================================================================
    // LatchInitialState() — 首次运行时绑定 ego 初始状态
    // ========================================================================
    void LatchInitialState(const ActorState& ego)
    {
        if (latched_) return;

        current_speed_ = ego.speed;
        current_s_     = ego.s;
        current_road_  = ego.road_id;
        current_lane_  = ego.lane_id;

        // 有全局路由时, 匹配初始位置到对应 segment, 并查找最近的全局路径点
        if (!route_segments_.empty())
        {
            current_segment_idx_ = 0;
            for (std::size_t i = 0; i < route_segments_.size(); ++i)
            {
                const auto& seg = route_segments_[i];
                if (seg.road_id != current_road_) continue;
                const double s_min = std::min(seg.s_start, seg.s_end);
                const double s_max = std::max(seg.s_start, seg.s_end);
                if (current_s_ >= s_min - 1.0 && current_s_ <= s_max + 1.0)
                {
                    current_segment_idx_ = i;
                    if (seg.lane_id != 0)
                        current_lane_ = seg.lane_id;
                    break;
                }
            }

            // 查找 ego 的世界坐标 (x,y) 对应全局路径上的最近点索引
            closest_global_path_idx_ = FindClosestGlobalPathIndex(ego.x, ego.y);

            std::fprintf(stderr,
                         "[RSimDriver] LatchInitialState: ego (%.2f, %.2f) → "
                         "route seg[%zu] road=%lld lane=%d s=%.2f, "
                         "closest worldPt[%zu] (%.2f, %.2f)\n",
                         ego.x, ego.y,
                         current_segment_idx_,
                         static_cast<long long>(current_road_),
                         current_lane_, current_s_,
                         closest_global_path_idx_,
                         global_path_world_points_.empty() ? 0.0
                             : global_path_world_points_[closest_global_path_idx_].x,
                         global_path_world_points_.empty() ? 0.0
                             : global_path_world_points_[closest_global_path_idx_].y);
        }

        latched_ = true;
    }

    ActorState BuildEgoView(const ActorState& ego) const
    {
        ActorState egoView = ego;
        egoView.s       = current_s_;
        egoView.t       = current_d_;
        egoView.speed   = current_speed_;
        egoView.road_id = current_road_;
        egoView.lane_id = current_lane_;
        return egoView;
    }

    // ========================================================================
    // BuildPlannerFrame() — 构建每帧的规划输入
    // ========================================================================
    // 参考线构建策略:
    //   - 有全局路由 → 从 global_path_world_points_ 中截取 ego 前方 lookahead 米
    //                  → ProvideFromWorldPoints() 直接用世界坐标建参考线
    //   - 无全局路由 → Provide() 在当前 lane 中心线采样 (fallback)
    PlannerFrame BuildPlannerFrame(const TickContext& ctx, const ActorState& ego)
    {
        PlannerFrame frame;
        frame.egoView = BuildEgoView(ego);

        const double laneWidth = map_.GetLaneWidth(current_road_, current_s_, current_lane_);
        const double decelForLookahead = std::max(max_decel_, 1e-3);
        const double lookahead =
            std::max(50.0, current_speed_ * current_speed_ / (2.0 * decelForLookahead) + 20.0);

        // ---- 感知扫描 ----
        frame.scan = perception_.Scan(frame.egoView, ctx.actors, lateral_dist_, lookahead, laneWidth);

        // ---- 车道边界 ----
        const int leftLane  = map_.GetLeftLane(current_road_, current_s_, current_lane_);
        const int rightLane = map_.GetRightLane(current_road_, current_s_, current_lane_);
        const bool sameDirLeft  = IsSameDirectionLane(leftLane);
        const bool sameDirRight = IsSameDirectionLane(rightLane);

        frame.reference.roadId          = current_road_;
        frame.reference.laneId          = current_lane_;
        frame.reference.laneWidth       = laneWidth;
        frame.reference.leftBound       = sameDirLeft  ? +laneWidth * 1.5 : +laneWidth * 0.4;
        frame.reference.rightBound      = sameDirRight ? -laneWidth * 1.5 : -laneWidth * 0.4;
        frame.reference.hasSameDirLeft  = sameDirLeft;
        frame.reference.hasSameDirRight = sameDirRight;
        frame.reference.distToJunction  = map_.DistToJunctionAhead(current_road_, current_s_);

        // ---- 构建参考线 ----
        if (!global_path_world_points_.empty())
        {
            // ================================================================
            // 路径A: 从全局路径世界坐标中截取并构建参考线
            // ================================================================
            // 步骤:
            //   1. 查找 ego 当前位置在全局路径上的最近点索引
            //   2. 从该索引开始向前提取世界坐标点子集 (长约 lookahead 米)
            //   3. 调用 ProvideFromWorldPoints() 做 chord-length 参数化 + 平滑 + 导数
            //   4. 返回的 ReferenceLine 中 s 锚定到 anchorS (= current_s_ 的路段偏移)
            //
            // 注意: ProvideFromWorldPoints 内部不再调用 TrackToWorld(),
            //       直接用世界坐标点构建参考线。

            // 更新 ego 在全局路径上的最近点索引 (ego 在持续前进)
            closest_global_path_idx_ = FindClosestGlobalPathIndex(
                ego.x, ego.y, closest_global_path_idx_);

            // 从 closest_global_path_idx_ 附近开始向前提取 lookahead 米的世界坐标点。
            // 接近终点时, 最近点之后可能不足 minPoints, 需要向后补几个历史点,
            // 否则最后几帧会因为 subset 点数不足而构建参考线失败。
            const size_t minRefPoints = std::max<size_t>(
                2, static_cast<size_t>(std::ceil(refLineProvider_.config.minPoints)));
            size_t subsetStartIdx = closest_global_path_idx_;
            if (global_path_world_points_.size() - subsetStartIdx < minRefPoints)
            {
                subsetStartIdx =
                    (global_path_world_points_.size() > minRefPoints)
                        ? global_path_world_points_.size() - minRefPoints
                        : 0;
            }

            std::vector<rsim_driver::WorldPoint> subset;
            subset.reserve(static_cast<size_t>(lookahead / 0.8) + 10);

            double accumulated = 0.0;
            for (size_t i = subsetStartIdx;
                 i < global_path_world_points_.size();
                 ++i)
            {
                subset.push_back(global_path_world_points_[i]);
                if (i > subsetStartIdx)
                {
                    double dx = global_path_world_points_[i].x
                              - global_path_world_points_[i - 1].x;
                    double dy = global_path_world_points_[i].y
                              - global_path_world_points_[i - 1].y;
                    accumulated += std::sqrt(dx * dx + dy * dy);
                }
                if (accumulated >= lookahead && subset.size() >= minRefPoints)
                    break;
            }

            // 计算 anchorS:
            //   当前 current_s_ 是 ego 在 road 上的 Frenet s。
            //   锚定第一个参考线点的 s 为: 当前 road-local s 减去 ego 距离最近全局路径
            //   点的 '累计弦长偏移'。这样参考线的 s 参数与 ExecutePlan 中的 current_s_
            //   保持一致, AdvanceRouterIfSegmentExhausted 才能正确判断 segment 切换。
            //
            double backtrack = 0.0;
            for (size_t i = subsetStartIdx + 1;
                 i <= closest_global_path_idx_ &&
                 i < global_path_world_points_.size();
                 ++i)
            {
                const double dx = global_path_world_points_[i].x
                                - global_path_world_points_[i - 1].x;
                const double dy = global_path_world_points_[i].y
                                - global_path_world_points_[i - 1].y;
                backtrack += std::sqrt(dx * dx + dy * dy);
            }
            const double anchorS = current_s_ - backtrack;

            frame.refLine = refLineProvider_.ProvideFromWorldPoints(
                subset, anchorS, lookahead);
            frame.usingGlobalRoute = true;

            if (enable_debug_log_)
            {
                std::fprintf(stderr,
                             "[RSimDriver] BuildPlannerFrame: [全局路由-世界坐标模式] "
                             "closestIdx=%zu subsetStart=%zu subset=%zu "
                             "anchorS=%.2f lookahead=%.2f refLine=%s\n",
                             closest_global_path_idx_, subsetStartIdx,
                             subset.size(), anchorS, lookahead,
                             frame.refLine ? "ok" : "FAILED");
            }
        }
        else
        {
            // ================================================================
            // 路径B: Fallback — 单路段 lane-following
            // ================================================================
            frame.refLine = refLineProvider_.Provide(
                map_, current_road_, current_lane_, current_s_, lookahead);
            frame.usingGlobalRoute = false;

            if (enable_debug_log_ || route_fetch_done_.load())
            {
                std::fprintf(stderr,
                             "[RSimDriver] BuildPlannerFrame: [WARNING 单路段Fallback] "
                             "全局路由未获取, road=%lld lane=%d startS=%.2f refLine=%s\n",
                             static_cast<long long>(current_road_), current_lane_,
                             current_s_,
                             frame.refLine ? "ok" : "FAILED");
            }
        }

        // ---- 障碍物坐标变换 ----
        frame.planSSign = (current_lane_ > 0) ? -1.0 : +1.0;
        frame.numPlanObstacles = frame.scan.numObstacles;
        for (int i = 0; i < frame.numPlanObstacles; ++i)
        {
            frame.planObstacles[i] = frame.scan.obstacles[i];
            frame.planObstacles[i].s = frame.planSSign * frame.scan.obstacles[i].s;
        }

        // ---- 跨路段障碍物检测 ----
        const double crossRoadRange = 30.0;
        const double cosH = std::cos(ego.h);
        const double sinH = std::sin(ego.h);
        for (const auto& other : ctx.actors)
        {
            if (other.id == ego.id) continue;
            if (other.road_id == current_road_) continue;

            const double dx = other.x - ego.x;
            const double dy = other.y - ego.y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > crossRoadRange) continue;

            const double longDist = dx * cosH + dy * sinH;
            const double latDist  = -dx * sinH + dy * cosH;

            if (frame.numPlanObstacles < rsim_driver::ScanResult::MAX_OBSTACLES)
            {
                rsim_driver::Obstacle& obs = frame.planObstacles[frame.numPlanObstacles];
                obs.objectIndex = -1;
                obs.s      = frame.planSSign * (current_s_ + longDist);
                obs.d      = current_d_ + latDist;
                obs.speed  = other.speed;
                obs.length = other.length;
                obs.width  = other.width;
                ++frame.numPlanObstacles;
            }
        }

        // ---- 设置当前 Frenet 状态 ----
        frame.currentState.s    = frame.planSSign * current_s_;
        frame.currentState.s_d  = current_speed_;
        frame.currentState.s_dd = prev_accel_;
        frame.currentState.d    = current_d_;
        frame.currentState.d_d  = current_d_speed_;
        frame.currentState.d_dd = current_d_accel_;

        // 保存 refLine 裸指针, 供 BuildActorUpdate 做 Frenet→World 投影
        // (frame 是 Step() 的局部变量, 生命周期覆盖到 BuildActorUpdate 调用)
        last_ref_line_ = frame.refLine.get();

        return frame;
    }

    bool IsSameDirectionLane(int laneId) const
    {
        if (laneId == 0) return false;
        return (current_lane_ < 0 && laneId < 0) || (current_lane_ > 0 && laneId > 0);
    }

    // ========================================================================
    // PlanFrame()
    // ========================================================================
    PlannerExecution PlanFrame(const TickContext& ctx, const PlannerFrame& frame)
    {
        rsim_driver::PlannerInput input;
        input.ego           = frame.currentState;
        input.desiredSpeed  = set_speed_;
        input.simTime       = ctx.sim_time;
        input.planningDt    = ctx.time_step;
        input.horizonTime   = planning_horizon_sec_;
        input.referenceLine = frame.refLine.get();
        input.obstacles     = frame.planObstacles;
        input.numObstacles  = frame.numPlanObstacles;

        const rsim_driver::PlannerOutput output = planner_->Plan(input);

        if (enable_debug_log_)
        {
            std::fprintf(stderr,
                         "[RSimDriver] PlanFrame: planner=%s mode=%s valid=%d cost=%.3f points=%d\n",
                         planner_type_.c_str(),
                         frame.usingGlobalRoute ? "全局路由(世界坐标)" : "单路段fallback",
                         output.valid ? 1 : 0,
                         output.frenet.cost,
                         output.trajectory.numPoints);
        }

        PlannerExecution execution;
        execution.valid      = output.valid;
        execution.frenet     = output.frenet;
        execution.trajectory = output.trajectory;
        return execution;
    }

    // ========================================================================
    // ExecutePlan()
    // ========================================================================
    void ExecutePlan(const PlannerExecution& plan, const PlannerFrame& frame, double dt)
    {
        if (!plan.valid || plan.trajectory.numPoints < 2)
        {
            ApplyFallbackBrake(frame.reference.leftBound, frame.reference.rightBound, dt);
            AdvanceRouterIfSegmentExhausted();
            return;
        }

        const double execT = std::min(std::max(dt, 0.0), plan.frenet.T);
        current_s_       = frame.planSSign * plan.frenet.EvalS(execT);
        current_speed_   = std::max(0.0, plan.frenet.EvalSdot(execT));
        prev_accel_      = plan.frenet.EvalSddot(execT);
        current_d_       = plan.frenet.EvalD(execT);
        current_d_speed_ = plan.frenet.EvalDdot(execT);
        current_d_accel_ = plan.frenet.EvalDddot(execT);

        AdvanceRouterIfSegmentExhausted();
    }

    // ========================================================================
    // AdvanceRouterIfSegmentExhausted()
    // ========================================================================
    // 当 ego 驶出当前 route segment 的 s 范围时, 自动切换到下一个 segment。
    void AdvanceRouterIfSegmentExhausted()
    {
        if (route_segments_.empty()) return;

        while (current_segment_idx_ + 1 < route_segments_.size())
        {
            const auto& seg = route_segments_[current_segment_idx_];
            const double progress = (current_s_ - seg.s_start) * seg.s_sign;
            const double seg_len  = std::fabs(seg.s_end - seg.s_start);

            if (progress < seg_len) break;

            const double leftover = progress - seg_len;
            ++current_segment_idx_;
            const auto& next = route_segments_[current_segment_idx_];

            current_road_ = next.road_id;
            if (next.lane_id != 0)
                current_lane_ = next.lane_id;
            current_s_ = next.s_start + leftover * next.s_sign;

            std::fprintf(stderr,
                         "[RSimDriver] Router: 进入 segment[%zu] road=%lld lane=%d s=%.2f\n",
                         current_segment_idx_,
                         static_cast<long long>(current_road_),
                         current_lane_, current_s_);
        }
    }

    void ApplyFallbackBrake(double leftBound, double rightBound, double dt)
    {
        const double brake = std::min(max_decel_, std::max(0.0, current_speed_ / std::max(dt, 1e-3)));
        current_speed_ = std::max(0.0, current_speed_ - brake * dt);
        prev_accel_ = current_speed_ > 0.0 ? -brake : 0.0;
        current_d_speed_ = 0.0;
        current_d_accel_ = 0.0;
        current_d_ = std::max(rightBound, std::min(leftBound, current_d_));
    }

    // ========================================================================
    // BuildActorUpdate() — 将 Frenet (s,d) 转回世界坐标发给仿真器
    // ========================================================================
    // 使用 ReferenceLine::Eval(s, d) 直接做 Frenet→World 投影,
    // 因为参考线是从世界坐标构建的, Eval 即返回世界坐标, 不再需要 TrackToWorld。
    //
    // 但如果参考线无效 (规划失败 fallback), 仍然通过 MapHelper 做转换。
    ActorUpdate BuildActorUpdate(int32_t actorId) const
    {
        rsim_driver::WorldPose pose;

        // 优先使用最近一帧的参考线做 Frenet→World 投影
        if (last_ref_line_)
        {
            double x, y, h;
            if (last_ref_line_->Eval(current_s_, current_d_, &x, &y, &h))
            {
                pose.x     = x;
                pose.y     = y;
                pose.h     = h;
                pose.valid = true;
            }
        }

        // 参考线投影失败, 回退到 MapHelper
        if (!pose.valid)
        {
            if (!route_segments_.empty() &&
                current_segment_idx_ < route_segments_.size())
            {
                const double t = route_segments_[current_segment_idx_].t + current_d_;
                pose = map_.TrackToWorld(current_road_, current_s_, t);
            }
            else
            {
                pose = map_.LaneToWorld(current_road_, current_lane_, current_s_, current_d_);
            }
        }

        ActorUpdate update {};
        update.actor_id = actorId;
        update.speed = current_speed_;
        update.speed_valid = 1;
        if (pose.valid)
        {
            update.x = pose.x;
            update.y = pose.y;
            update.z = pose.z;
            update.h = pose.h + std::atan2(current_d_speed_, std::max(0.1, current_speed_));
            update.position_valid = 1;
        }
        return update;
    }

    // ========================================================================
    // 成员变量
    // ========================================================================
    std::vector<int32_t> controlled_ids_;
    rsim_driver::MapHelper               map_;
    rsim_driver::PerceptionHelper        perception_;
    std::unique_ptr<rsim_driver::IPlanner> planner_;
    rsim_driver::ReferenceLineProvider   refLineProvider_;
    rsim_driver::TrafficRuleHandler      trafficRule_;

    // ---- 全局路由 — 路段索引 (用于 road/lane tracking) ----
    std::vector<RuntimeRouteSegment>     route_segments_;
    std::size_t                          current_segment_idx_ = 0;

    // ---- 全局路由 — 世界坐标点 (用于参考线构建) ----
    // 从 Route waypoint 直接提取 (x_val, y_val, heading), 插值加密到 ~1m 间距,
    // 传给 ProvideFromWorldPoints() 直接构建参考线, 不经过道路参考线映射。
    std::vector<rsim_driver::WorldPoint> global_path_world_points_;
    size_t closest_global_path_idx_ = 0; // ego 在 global_path 上的最近点索引

    // ---- 最近一帧的参考线 (用于 BuildActorUpdate 中的 Frenet→World 投影) ----
    // 因为参考线是每帧新构建的 unique_ptr, BuildActorUpdate 在 ExecutePlan 之后,
    // 需要保留到 BuildActorUpdate 使用完毕。
    // 简化方案: BuildPlannerFrame 创建的 refLine 存在 PlannerFrame 中,
    // 在 Step() 中 PlannerFrame 是局部变量, 生命周期覆盖到 BuildActorUpdate。
    // 这里存一个裸指针引用, 在 BuildPlannerFrame 末尾更新。
    mutable const rsim_driver::ReferenceLine* last_ref_line_ = nullptr;

    // ---- SceneRunner RPC ----
    std::string sr_host_     = "127.0.0.1";
    uint16_t    sr_port_     = 9110;
    std::string entity_name_ = "ego";
    std::string route_xosc_path_;
    std::string route_source_ = "none";

    std::thread             route_fetcher_;
    std::atomic<bool>       route_fetch_done_{false};
    std::mutex              pending_route_mtx_;
    rsim::rpc::Route        pending_route_;
    bool                    pending_route_ok_ = false;
    bool                    route_installed_  = false;

    // ---- 规划器配置 ----
    std::string planner_type_ = "sampling";
    double  set_speed_    = 10.0;
    double  max_accel_    = 3.0;
    double  max_decel_    = 4.0;
    double  lateral_dist_ = 2.0;
    double  planning_horizon_sec_ = 5.0;
    bool    enable_debug_log_ = false;

    // ---- ego 运行状态 ----
    double  current_speed_   = 0.0;
    double  prev_accel_      = 0.0;
    double  current_s_       = 0.0;  // road-local s (用于 segment tracking)
    double  current_d_       = 0.0;
    double  current_d_speed_ = 0.0;
    double  current_d_accel_ = 0.0;
    int32_t current_road_    = 0;
    int     current_lane_    = 0;
    bool    latched_         = false;
    bool    map_loaded_      = false;
};

}  // namespace

extern "C" IPluginController* CreateController(const char* name)
{
    (void) name;
    return new RSimDriverPlugin();
}

extern "C" void DestroyController(IPluginController* p)
{
    delete p;
}
