/*
 * ReferenceLineProvider implementation.
 */
#include "ReferenceLineProvider.hpp"
#include "ReferenceLine.hpp"
#include "MapHelper.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace rsim_driver
{
namespace
{

// 5-point moving average on x and y separately.  Modifies points in place.
void MovingAverageSmooth(std::vector<ReferencePoint>& pts, int half)
{
    if (half < 1 || pts.size() < 3)
        return;

    std::vector<double> xBuf(pts.size());
    std::vector<double> yBuf(pts.size());

    for (size_t i = 0; i < pts.size(); ++i)
    {
        size_t lo = (i >= static_cast<size_t>(half)) ? i - half : 0;
        size_t hi = std::min(pts.size() - 1, i + static_cast<size_t>(half));
        double sumX = 0.0, sumY = 0.0;
        size_t n = 0;
        for (size_t j = lo; j <= hi; ++j, ++n)
        {
            sumX += pts[j].x;
            sumY += pts[j].y;
        }
        xBuf[i] = sumX / static_cast<double>(n);
        yBuf[i] = sumY / static_cast<double>(n);
    }

    for (size_t i = 0; i < pts.size(); ++i)
    {
        pts[i].x = xBuf[i];
        pts[i].y = yBuf[i];
    }
}

// Recompute heading and curvature from (x, y, s).
void RecomputeDerivatives(std::vector<ReferencePoint>& pts)
{
    if (pts.size() < 2)
    {
        for (auto& p : pts)
        {
            p.hdg = 0.0;
            p.k   = 0.0;
            p.dk  = 0.0;
        }
        return;
    }

    // Heading from central differences (forward/backward at boundaries).
    std::vector<double> hdg(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
    {
        if (i == 0)
        {
            hdg[i] = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
        }
        else if (i == pts.size() - 1)
        {
            hdg[i] = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
        }
        else
        {
            hdg[i] = std::atan2(pts[i + 1].y - pts[i - 1].y, pts[i + 1].x - pts[i - 1].x);
        }
        pts[i].hdg = hdg[i];
    }

    // Curvature: k = dh/ds
    for (size_t i = 0; i < pts.size(); ++i)
    {
        if (i == 0 && pts.size() > 1)
        {
            double dh = hdg[1] - hdg[0];
            while (dh > M_PI)  dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            double ds = pts[1].s - pts[0].s;
            pts[i].k = (ds > 1e-6) ? dh / ds : 0.0;
        }
        else if (i == pts.size() - 1 && pts.size() > 1)
        {
            double dh = hdg[i] - hdg[i - 1];
            while (dh > M_PI)  dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            double ds = pts[i].s - pts[i - 1].s;
            pts[i].k = (ds > 1e-6) ? dh / ds : 0.0;
        }
        else if (pts.size() > 2)
        {
            double dh = hdg[i + 1] - hdg[i - 1];
            while (dh > M_PI)  dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            double ds = pts[i + 1].s - pts[i - 1].s;
            pts[i].k = (ds > 1e-6) ? dh / ds : 0.0;
        }
        else
        {
            pts[i].k = 0.0;
        }
    }

    // Curvature derivative: dk = (k[i+1] - k[i-1]) / (2*ds) with analogous boundary handling.
    for (size_t i = 0; i < pts.size(); ++i)
    {
        if (i == 0 && pts.size() > 1)
        {
            double ds = pts[1].s - pts[0].s;
            pts[i].dk = (ds > 1e-6) ? (pts[1].k - pts[0].k) / ds : 0.0;
        }
        else if (i == pts.size() - 1 && pts.size() > 1)
        {
            double ds = pts[i].s - pts[i - 1].s;
            pts[i].dk = (ds > 1e-6) ? (pts[i].k - pts[i - 1].k) / ds : 0.0;
        }
        else if (pts.size() > 2)
        {
            double ds = pts[i + 1].s - pts[i - 1].s;
            pts[i].dk = (ds > 1e-6) ? (pts[i + 1].k - pts[i - 1].k) / ds : 0.0;
        }
        else
        {
            pts[i].dk = 0.0;
        }
    }
}

}  // namespace

std::unique_ptr<ReferenceLine> ReferenceLineProvider::Provide(
    const MapHelper& map, int64_t roadId, int laneId,
    double startS, double lookahead) const
{
    const double step = std::max(0.1, config.sampleStep);
    const double endS = startS + std::max(1.0, lookahead);

    // --- 1. Raw sampling along lane centreline ---
    std::vector<ReferencePoint> raw;
    for (double s = startS; s <= endS + 1e-6; s += step)
    {
        WorldPose pose = map.LaneToWorld(roadId, laneId, s, /*lateralOffset=*/0.0);
        if (!pose.valid)
            break;

        ReferencePoint pt;
        pt.x   = pose.x;
        pt.y   = pose.y;
        pt.hdg = pose.h;
        pt.k   = 0.0;
        pt.dk  = 0.0;
        pt.s   = 0.0;  // will be recomputed as chord-length
        raw.push_back(pt);
    }

    if (static_cast<double>(raw.size()) < config.minPoints)
        return nullptr;

    // --- 2. Chord-length parameterisation ---
    raw[0].s = startS;  // anchor first point to the input startS
    for (size_t i = 1; i < raw.size(); ++i)
    {
        double dx = raw[i].x - raw[i - 1].x;
        double dy = raw[i].y - raw[i - 1].y;
        raw[i].s = raw[i - 1].s + std::sqrt(dx * dx + dy * dy);
    }

    // --- 3. Smoothing ---
    for (int pass = 0; pass < config.smoothPasses; ++pass)
        MovingAverageSmooth(raw, config.smoothHalf);

    // --- 4. Recompute heading, curvature, dk from smoothed (x, y, s) ---
    RecomputeDerivatives(raw);

    auto result = std::make_unique<ReferenceLine>();
    result->points = std::move(raw);
    return result;
}

std::unique_ptr<ReferenceLine> ReferenceLineProvider::ProvideForRoute(
    const MapHelper& map,
    const std::vector<RouteLaneSegment>& segments,
    std::size_t start_idx,
    double start_s,
    double lookahead) const
{
    if (segments.empty() || start_idx >= segments.size())
        return nullptr;

    const double step      = std::max(0.1, config.sampleStep);
    const double targetLen = std::max(1.0, lookahead);

    // --- 1. Raw sampling across segments. Chord-length s is anchored at
    // `start_s` (so the first point shares the ego's road-local s). After the
    // first segment we just keep accumulating chord length; values past the
    // first segment are no longer meaningful as road-local s but the planner
    // only relies on the consistent monotonic parameterisation. ---
    std::vector<ReferencePoint> raw;
    double accumS = start_s;
    std::size_t idx = start_idx;
    double cursor = start_s;
    bool first = true;

    while (idx < segments.size())
    {
        const auto& seg = segments[idx];
        // Distance left in this segment along its s direction (can be 0 or
        // even slightly negative if start_s started ahead of s_end).
        double segRemain = (seg.s_end - cursor) * seg.s_sign;

        // Always sample at least the entry point of the (sub)segment so that
        // the reference line includes a point at the current ego s on the
        // current road, even if the segment is very short.
        bool entryEmitted = false;

        while ((!entryEmitted) ||
               (segRemain > 0.0 &&
                (raw.size() < 2 ||
                 raw.back().s - raw.front().s < targetLen)))
        {
            WorldPose pose = map.TrackToWorld(seg.road_id, cursor, seg.t);
            if (!pose.valid)
                break;

            ReferencePoint pt;
            pt.x   = pose.x;
            pt.y   = pose.y;
            pt.hdg = pose.h;
            pt.k   = 0.0;
            pt.dk  = 0.0;
            if (first)
            {
                pt.s  = accumS;
                first = false;
            }
            else
            {
                double dx = pt.x - raw.back().x;
                double dy = pt.y - raw.back().y;
                accumS += std::sqrt(dx * dx + dy * dy);
                pt.s   = accumS;
            }
            raw.push_back(pt);
            entryEmitted = true;

            cursor    += step * seg.s_sign;
            segRemain -= step;
        }

        if (raw.size() >= 2 &&
            raw.back().s - raw.front().s >= targetLen)
            break;

        // Move to next segment, anchoring at its s_start.
        ++idx;
        if (idx < segments.size())
            cursor = segments[idx].s_start;
    }

    if (static_cast<double>(raw.size()) < config.minPoints)
        return nullptr;

    // --- 2. Smoothing ---
    for (int pass = 0; pass < config.smoothPasses; ++pass)
        MovingAverageSmooth(raw, config.smoothHalf);

    // --- 3. Recompute heading / curvature / dk from smoothed (x, y, s). ---
    RecomputeDerivatives(raw);

    auto result = std::make_unique<ReferenceLine>();
    result->points = std::move(raw);
    return result;
}

std::unique_ptr<ReferenceLine> ReferenceLineProvider::ProvideFromWorldPoints(
    const std::vector<WorldPoint>& worldPoints,
    double anchorS,
    double lookahead) const
{
    if (worldPoints.size() < 2)
        return nullptr;

    const double targetLen = std::max(1.0, lookahead);

    // ---- 1. 将 WorldPoint 转为 ReferencePoint, 以 chord-length 参数化 ----
    // 第一个点的 s 锚定到 anchorS, 后续点累加弦长距离
    std::vector<ReferencePoint> raw;
    raw.reserve(worldPoints.size());

    double accumS = anchorS;
    {
        ReferencePoint pt;
        pt.x   = worldPoints[0].x;
        pt.y   = worldPoints[0].y;
        pt.hdg = worldPoints[0].hdg;
        pt.k   = 0.0;
        pt.dk  = 0.0;
        pt.s   = accumS;
        raw.push_back(pt);
    }

    for (size_t i = 1; i < worldPoints.size(); ++i)
    {
        double dx = worldPoints[i].x - worldPoints[i - 1].x;
        double dy = worldPoints[i].y - worldPoints[i - 1].y;
        accumS += std::sqrt(dx * dx + dy * dy);

        ReferencePoint pt;
        pt.x   = worldPoints[i].x;
        pt.y   = worldPoints[i].y;
        pt.hdg = worldPoints[i].hdg;
        pt.k   = 0.0;
        pt.dk  = 0.0;
        pt.s   = accumS;
        raw.push_back(pt);

        // 已累积足够长度, 停止
        if (accumS - anchorS >= targetLen && raw.size() >= config.minPoints)
            break;
    }

    if (static_cast<double>(raw.size()) < config.minPoints)
        return nullptr;

    // ---- 2. 平滑 ----
    for (int pass = 0; pass < config.smoothPasses; ++pass)
        MovingAverageSmooth(raw, config.smoothHalf);

    // ---- 3. 从平滑后的 (x, y, s) 重新计算 heading / curvature / dk ----
    RecomputeDerivatives(raw);

    auto result = std::make_unique<ReferenceLine>();
    result->points = std::move(raw);
    return result;
}

}  // namespace rsim_driver
