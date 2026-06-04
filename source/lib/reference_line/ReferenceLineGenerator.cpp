/*
 * ReferenceLineGenerator implementation.
 */
#include "ReferenceLineGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rsim_driver
{
    namespace
    {
        double NormalizeAngle(double angle)
        {
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        }

        double DistanceSquared(const WorldPoint &point, double x, double y)
        {
            const double dx = point.x - x;
            const double dy = point.y - y;
            return dx * dx + dy * dy;
        }

        ReferencePoint MakeReferencePoint(double x, double y)
        {
            ReferencePoint point;
            point.x = x;
            point.y = y;
            point.hdg = 0.0;
            point.k = 0.0;
            point.dk = 0.0;
            point.s = 0.0;
            return point;
        }

        ReferencePoint MakeReferencePoint(const WorldPoint &point)
        {
            return MakeReferencePoint(point.x, point.y);
        }

    } // namespace

    ReferenceLineGenerator::ReferenceLineGenerator()
    {
        smoother_.config = Generateconfig.smoother;
    }
    ReferenceLineGenerator::ReferenceLineGenerator(const GenerateConfig &inputConfig)
        : Generateconfig(inputConfig)
    {
    }

    std::unique_ptr<ReferenceLine> ReferenceLineGenerator::Generate(
        const std::vector<WorldPoint> &globalPath,
        double egoX,
        double egoY)
    {
        if (globalPath.size() < static_cast<std::size_t>(std::max(1, Generateconfig.minPoints)))
            return nullptr;

        smoother_.config = Generateconfig.smoother;

        const std::size_t matchIndex = FindMatchIndex(globalPath, egoX, egoY);
        last_projection_index_ = matchIndex;
        has_projection_ = true;

        std::vector<ReferencePoint> raw = BuildRawWindow(globalPath, matchIndex);
        if (raw.size() < static_cast<std::size_t>(std::max(1, Generateconfig.minPoints)))
            return nullptr;

        std::vector<ReferencePoint> smoothed;
        if (!smoother_.Smooth(raw, &smoothed))
            return nullptr;
        ReferenceLineGenerator::RecomputeGeometry(&smoothed);

        auto result = std::make_unique<ReferenceLine>();
        result->points = std::move(smoothed);
        return result;
    }

    std::size_t ReferenceLineGenerator::FindMatchIndex(
        const std::vector<WorldPoint> &globalPath,
        double egoX,
        double egoY) const
    {
        const std::size_t n = globalPath.size();
        if (n == 0)
            return 0;

        const std::size_t begin =
            has_projection_ ? std::min(last_projection_index_, n - 1) : 0;
        const std::size_t confirmCount =
            std::max<std::size_t>(1, Generateconfig.matchConfirmForwardPoints);

        std::size_t bestIndex = begin;
        double bestDistance = std::numeric_limits<double>::infinity();
        std::size_t consecutiveFarther = 0;

        for (std::size_t i = begin; i < n; ++i)
        {
            const double distance = DistanceSquared(globalPath[i], egoX, egoY);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = i;
                consecutiveFarther = 0;
                continue;
            }
            ++consecutiveFarther;
            if (consecutiveFarther >= confirmCount)
                break;
        }

        return bestIndex;
    }

    std::vector<ReferencePoint> ReferenceLineGenerator::BuildRawWindow(
        const std::vector<WorldPoint> &globalPath,
        std::size_t matchIndex)
    {
        const std::size_t n = globalPath.size();
        if (n == 0)
            return {};

        const std::size_t backward =
            static_cast<std::size_t>(std::max(0, Generateconfig.backwardPoints));
        const std::size_t forward =
            static_cast<std::size_t>(std::max(0, Generateconfig.forwardPoints));
        const std::size_t targetCount = backward + 1 + forward;

        const std::size_t center = std::min(matchIndex, n - 1);
        std::size_t start = (center > backward) ? center - backward : 0;
        std::size_t end = std::min(n - 1, center + forward);

        std::vector<ReferencePoint> raw;
        raw.reserve(targetCount);
        for (std::size_t i = start; i <= end; ++i)
            raw.push_back(MakeReferencePoint(globalPath[i]));
        return raw;
    }
    void ReferenceLineGenerator::RecomputeGeometry(std::vector<ReferencePoint> *points)
    {
        if (points == nullptr || points->empty())
            return;

        std::vector<ReferencePoint> &pts = *points;
        const double anchorS = pts.front().s;
        pts.front().s = anchorS;
        for (std::size_t i = 1; i < pts.size(); ++i)
        {
            const double dx = pts[i].x - pts[i - 1].x;
            const double dy = pts[i].y - pts[i - 1].y;
            pts[i].s = pts[i - 1].s + std::sqrt(dx * dx + dy * dy);
        }

        if (pts.size() < 2)
        {
            pts.front().hdg = 0.0;
            pts.front().k = 0.0;
            pts.front().dk = 0.0;
            return;
        }

        std::vector<double> heading(pts.size(), 0.0);
        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            if (i == 0)
            {
                heading[i] = std::atan2(pts[1].y - pts[0].y,
                                        pts[1].x - pts[0].x);
            }
            else if (i + 1 == pts.size())
            {
                heading[i] = std::atan2(pts[i].y - pts[i - 1].y,
                                        pts[i].x - pts[i - 1].x);
            }
            else
            {
                heading[i] = std::atan2(pts[i + 1].y - pts[i - 1].y,
                                        pts[i + 1].x - pts[i - 1].x);
            }
            pts[i].hdg = heading[i];
        }

        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            if (i == 0)
            {
                const double ds = pts[1].s - pts[0].s;
                pts[i].k = (ds > 1e-6) ? NormalizeAngle(heading[1] - heading[0]) / ds : 0.0;
            }
            else if (i + 1 == pts.size())
            {
                const double ds = pts[i].s - pts[i - 1].s;
                pts[i].k = (ds > 1e-6)
                               ? NormalizeAngle(heading[i] - heading[i - 1]) / ds
                               : 0.0;
            }
            else
            {
                const double ds = pts[i + 1].s - pts[i - 1].s;
                pts[i].k = (ds > 1e-6)
                               ? NormalizeAngle(heading[i + 1] - heading[i - 1]) / ds
                               : 0.0;
            }
        }

        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            if (i == 0)
            {
                const double ds = pts[1].s - pts[0].s;
                pts[i].dk = (ds > 1e-6) ? (pts[1].k - pts[0].k) / ds : 0.0;
            }
            else if (i + 1 == pts.size())
            {
                const double ds = pts[i].s - pts[i - 1].s;
                pts[i].dk = (ds > 1e-6) ? (pts[i].k - pts[i - 1].k) / ds : 0.0;
            }
            else
            {
                const double ds = pts[i + 1].s - pts[i - 1].s;
                pts[i].dk = (ds > 1e-6) ? (pts[i + 1].k - pts[i - 1].k) / ds : 0.0;
            }
        }
    }

} // namespace rsim_driver
