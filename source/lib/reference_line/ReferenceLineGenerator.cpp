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

        void RecomputeHeading(std::vector<ReferencePoint> &pts)
        {
            if (pts.empty())
                return;

            if (pts.size() < 2)
            {
                pts.front().hdg = 0.0;
                return;
            }

            for (std::size_t i = 0; i < pts.size(); ++i)
            {
                if (i == 0)
                {
                    pts[i].hdg = std::atan2(pts[1].y - pts[0].y,
                                            pts[1].x - pts[0].x);
                }
                else if (i + 1 == pts.size())
                {
                    pts[i].hdg = std::atan2(pts[i].y - pts[i - 1].y,
                                            pts[i].x - pts[i - 1].x);
                }
                else
                {
                    pts[i].hdg = std::atan2(pts[i + 1].y - pts[i - 1].y,
                                            pts[i + 1].x - pts[i - 1].x);
                }
            }
        }

        double SegmentProjectionRatio(const ReferencePoint &a,
                                      const ReferencePoint &b,
                                      const ReferencePoint &point)
        {
            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double len = vx * vx + vy * vy;
            if (len <= 1e-12)
                return 0.0;

            const double wx = point.x - a.x;
            const double wy = point.y - a.y;
            return std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / len));
        }

        void RecomputeCurvature(std::vector<ReferencePoint> &pts)
        {
            for (ReferencePoint &point : pts)
            {
                point.k = 0.0;
                point.dk = 0.0;
            }

            if (pts.size() < 2)
                return;

            for (std::size_t i = 0; i < pts.size(); ++i)
            {
                if (i == 0)
                {
                    const double ds = pts[1].s - pts[0].s;
                    pts[i].k = (std::fabs(ds) > 1e-6)
                                   ? NormalizeAngle(pts[1].hdg - pts[0].hdg) / ds
                                   : 0.0;
                }
                else if (i + 1 == pts.size())
                {
                    const double ds = pts[i].s - pts[i - 1].s;
                    pts[i].k = (std::fabs(ds) > 1e-6)
                                   ? NormalizeAngle(pts[i].hdg - pts[i - 1].hdg) / ds
                                   : 0.0;
                }
                else
                {
                    const double ds = pts[i + 1].s - pts[i - 1].s;
                    pts[i].k = (std::fabs(ds) > 1e-6)
                                   ? NormalizeAngle(pts[i + 1].hdg - pts[i - 1].hdg) / ds
                                   : 0.0;
                }
            }

            for (std::size_t i = 0; i < pts.size(); ++i)
            {
                if (i == 0)
                {
                    const double ds = pts[1].s - pts[0].s;
                    pts[i].dk = (std::fabs(ds) > 1e-6) ? (pts[1].k - pts[0].k) / ds : 0.0;
                }
                else if (i + 1 == pts.size())
                {
                    const double ds = pts[i].s - pts[i - 1].s;
                    pts[i].dk = (std::fabs(ds) > 1e-6) ? (pts[i].k - pts[i - 1].k) / ds : 0.0;
                }
                else
                {
                    const double ds = pts[i + 1].s - pts[i - 1].s;
                    pts[i].dk = (std::fabs(ds) > 1e-6) ? (pts[i + 1].k - pts[i - 1].k) / ds : 0.0;
                }
            }
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
        if (isfirst_)
        {
            path_matcher_detail::kMatchConfirmLookahead = Generateconfig.matchConfirmForwardPoints;
            isfirst_ = false;
        }
        else
        {
            path_matcher_detail::kMatchConfirmLookahead = Generateconfig.matchConfirmForwardPoints - 8;
        }

        const std::size_t matchIndex = FindMatchPointIndex(globalPath, egoX, egoY, last_match_point_index_);
        last_match_point_index_ = matchIndex;
        std::vector<ReferencePoint> raw = BuildRawWindow(globalPath, matchIndex);
        if (raw.size() < static_cast<std::size_t>(std::max(1, Generateconfig.minPoints)))
            return nullptr;

        std::vector<ReferencePoint> smoothed;
        if (!smoother_.Smooth(raw, &smoothed))
            return nullptr;
        ReferenceLineGenerator::RecomputeGeometry(&smoothed);

        cur_projection_point_ = FindProjectionPoint(smoothed, egoX, egoY);
        cur_projection_point_.s = 0.0;
        cur_projection_point_.k = 0.0;
        cur_projection_point_.dk = 0.0;

        ReferenceLineGenerator::RecomputeGeometry(&smoothed, cur_projection_point_);

        auto result = std::make_unique<ReferenceLine>();
        result->points = std::move(smoothed);
        return result;
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
        const std::size_t desiredCount = std::min(targetCount, n);

        const std::size_t center = std::min(matchIndex, n - 1);
        std::size_t start = (center > backward) ? center - backward : 0;
        std::size_t end = std::min(n - 1, center + forward);
        std::size_t count = end - start + 1;

        if (count < desiredCount)
        {
            std::size_t missing = desiredCount - count;
            const std::size_t rightAvailable = (n - 1) - end;
            const std::size_t extendRight = std::min(missing, rightAvailable);
            end += extendRight;
            missing -= extendRight;

            const std::size_t extendLeft = std::min(missing, start);
            start -= extendLeft;
        }

        std::vector<ReferencePoint> raw;
        raw.reserve(desiredCount);
        for (std::size_t i = start; i <= end; ++i)
            raw.push_back(MakeReferencePoint(globalPath[i]));
        return raw;
    }
    void ReferenceLineGenerator::RecomputeGeometry(std::vector<ReferencePoint> *points)
    {
        if (points == nullptr || points->empty())
            return;

        std::vector<ReferencePoint> &pts = *points;
        RecomputeHeading(pts);

        for (ReferencePoint &point : pts)
        {
            point.k = 0.0;
            point.dk = 0.0;
        }
    }

    void ReferenceLineGenerator::RecomputeGeometry(std::vector<ReferencePoint> *points,
                                                   const ReferencePoint &projectionPoint)
    {
        if (points == nullptr || points->empty())
            return;

        std::vector<ReferencePoint> &pts = *points;

        if (pts.size() < 2)
        {
            pts.front().s = 0.0;
            pts.front().k = 0.0;
            pts.front().dk = 0.0;
            return;
        }

        std::vector<double> cumulative(pts.size(), 0.0);
        for (std::size_t i = 1; i < pts.size(); ++i)
        {
            const double dx = pts[i].x - pts[i - 1].x;
            const double dy = pts[i].y - pts[i - 1].y;
            cumulative[i] = cumulative[i - 1] + std::sqrt(dx * dx + dy * dy);
        }

        std::size_t bestSegment = 0;
        double bestRatio = 0.0;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i + 1 < pts.size(); ++i)
        {
            const double ratio = SegmentProjectionRatio(pts[i], pts[i + 1], projectionPoint);
            const double projX = pts[i].x + ratio * (pts[i + 1].x - pts[i].x);
            const double projY = pts[i].y + ratio * (pts[i + 1].y - pts[i].y);
            const double dx = projectionPoint.x - projX;
            const double dy = projectionPoint.y - projY;
            const double distance = dx * dx + dy * dy;
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestSegment = i;
                bestRatio = ratio;
            }
        }

        const double segmentLength = cumulative[bestSegment + 1] - cumulative[bestSegment];
        const double projectionArcS = cumulative[bestSegment] + bestRatio * segmentLength;
        for (std::size_t i = 0; i < pts.size(); ++i)
            pts[i].s = cumulative[i] - projectionArcS;

        RecomputeCurvature(pts);
    }

} // namespace rsim_driver
