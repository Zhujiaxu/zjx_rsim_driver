/*
 * ReferenceLineProvider — 构建规划参考线 (planning reference line)。
 *
 * 提供三种构建方式:
 *   Provide()               — 从单个 lane 的中心线采样 (fallback 模式)
 *   ProvideForRoute()        — 从多路段 Route 通过 TrackToWorld 采样 (旧全局路由模式)
 *   ProvideFromWorldPoints() — 直接从世界坐标点构建 (新全局路由模式,
 *                               跳过道路参考线映射, 用 Route waypoint 的 x/y/h)
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace rsim_driver
{

class MapHelper;
struct ReferenceLine;
struct ReferencePoint;

// ---- Slim segment description for ProvideForRoute (旧接口, 保留兼容) ----
struct RouteLaneSegment
{
    int64_t road_id = 0;
    int     lane_id = 0;
    double  s_start = 0.0;
    double  s_end   = 0.0;
    double  s_sign  = 1.0;  // +1 along road s, -1 against
    double  t       = 0.0;  // lateral offset in road frame (route-provided)
};

// ---- 世界坐标点: 直接从 Route waypoint 提取, 不经过道路参考线映射 ----
// 由 SceneRunnerClient::GetRoute() 返回的 WayPoint 中提取 (x_val, y_val, heading),
// 经过线性插值加密到 ~1m 间距后传入 ProvideFromWorldPoints()。
struct WorldPoint
{
    double x   = 0.0;
    double y   = 0.0;
    double hdg = 0.0;  // 世界坐标系下的航向角 (rad)
};

class ReferenceLineProvider
{
public:
    struct Config
    {
        double sampleStep  = 1.0;   // sampling interval along lane s (m)
        int    smoothHalf  = 2;     // half-window for moving-average (2 → 5-pt)
        double minPoints   = 4.0;   // minimum number of points required
        int    smoothPasses = 1;    // number of smoothing passes
    };

    Config config;

    // ---- Fallback: 从单个 lane 中心线采样 ----
    std::unique_ptr<ReferenceLine> Provide(const MapHelper& map,
                                           int64_t roadId, int laneId,
                                           double startS, double lookahead) const;

    // ---- 旧全局路由模式: 从多路段 Route 通过 TrackToWorld(road,s,t) 采样 ----
    std::unique_ptr<ReferenceLine> ProvideForRoute(
        const MapHelper& map,
        const std::vector<RouteLaneSegment>& segments,
        std::size_t start_idx,
        double start_s,
        double lookahead) const;

    // ---- 新全局路由模式: 直接从世界坐标点构建参考线 ----
    // worldPoints 应已做线性插值加密到 ~1m 间距, 保证平滑质量。
    // anchorS 是第一个点的 s 值 (通常设为 ego 当前的 road-local s,
    // 以与 AdvanceRouterIfSegmentExhausted 中的 current_s_ 保持一致)。
    // lookahead 是需要的参考线长度 (米)。
    // 内部做 chord-length 参数化 → 平滑 → 导数重算。
    std::unique_ptr<ReferenceLine> ProvideFromWorldPoints(
        const std::vector<WorldPoint>& worldPoints,
        double anchorS,
        double lookahead) const;
};

}  // namespace rsim_driver
