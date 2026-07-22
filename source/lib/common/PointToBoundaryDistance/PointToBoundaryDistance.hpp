#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rsim_driver
{

    struct StPoint
    {
        double s = 0.0;
        double t = 0.0;
    };

    struct PointToBoundaryDistanceResult
    {
        int32_t id = 0;
        double distance = 0.0;
    };

    namespace point_to_boundary_distance_detail
    {

        constexpr double kDistanceEpsilon = 1e-9;

        struct StPolygon
        {
            StPoint p1;
            StPoint p2;
            StPoint p3;
            StPoint p4;
        };

        inline bool IsFinite(const StPoint &point)
        {
            return std::isfinite(point.s) && std::isfinite(point.t);
        }

        inline double Dot(const StPoint &lhs, const StPoint &rhs)
        {
            return lhs.s * rhs.s + lhs.t * rhs.t;
        }

        inline double Cross(const StPoint &lhs, const StPoint &rhs)
        {
            return lhs.s * rhs.t - lhs.t * rhs.s;
        }

        inline double Norm(const StPoint &point)
        {
            return std::hypot(point.s, point.t);
        }

        inline bool SamePositiveSign(double lhs, double rhs)
        {
            return lhs > 0.0 && rhs > 0.0;
        }

        inline bool SameNegativeSign(double lhs, double rhs)
        {
            return lhs < 0.0 && rhs < 0.0;
        }

        inline StPoint Subtract(const StPoint &lhs, const StPoint &rhs)
        {
            return {lhs.s - rhs.s, lhs.t - rhs.t};
        }

        inline bool PolygonIsFinite(const StPolygon &polygon)
        {
            return IsFinite(polygon.p1) &&
                   IsFinite(polygon.p2) &&
                   IsFinite(polygon.p3) &&
                   IsFinite(polygon.p4);
        }

        inline double SignedDoubleArea(const StPoint *points, std::size_t count)
        {
            double area = 0.0;
            for (std::size_t i = 0; i < count; ++i)
            {
                const StPoint &current = points[i];
                const StPoint &next = points[(i + 1) % count];
                area += current.s * next.t - next.s * current.t;
            }
            return area;
        }

        inline bool IsInsideConvexPolygon(const StPoint &point,
                                          const StPoint *points,
                                          std::size_t count)
        {
            bool hasPositive = false;
            bool hasNegative = false;
            for (std::size_t i = 0; i < count; ++i)
            {
                const StPoint &current = points[i];
                const StPoint &next = points[(i + 1) % count];
                const double cross =
                    Cross(Subtract(next, current), Subtract(point, current));
                if (std::fabs(cross) <= kDistanceEpsilon)
                    continue;
                hasPositive = hasPositive || cross > 0.0;
                hasNegative = hasNegative || cross < 0.0;
                if (hasPositive && hasNegative)
                    return false;
            }
            return true;
        }

        inline void NormalizeRange(double *lower, double *upper)
        {
            if (*upper < *lower)
                std::swap(*lower, *upper);
        }

        inline StPolygon MakePolygon(double tin,
                                     double tout,
                                     double sinmin,
                                     double sinmax,
                                     double soutmin,
                                     double soutmax)
        {
            NormalizeRange(&sinmin, &sinmax);
            NormalizeRange(&soutmin, &soutmax);

            return {
                {sinmin, tin},
                {sinmax, tin},
                {soutmin, tout},
                {soutmax, tout},
            };
        }

    } // namespace point_to_boundary_distance_detail

    inline double PointToLineSegmentDistance(const StPoint &point,
                                             const StPoint &segmentStart,
                                             const StPoint &segmentEnd)
    {
        using namespace point_to_boundary_distance_detail;

        if (!IsFinite(point) || !IsFinite(segmentStart) || !IsFinite(segmentEnd))
            return std::numeric_limits<double>::infinity();

        const StPoint a{segmentStart.t - point.t, segmentStart.s - point.s};
        const StPoint b{segmentEnd.t - point.t, segmentEnd.s - point.s};
        const StPoint c{segmentEnd.t - segmentStart.t,
                        segmentEnd.s - segmentStart.s};

        const double cNorm = Norm(c);
        if (cNorm <= kDistanceEpsilon)
            return Norm(a);

        const double ac = Dot(a, c);
        const double bc = Dot(b, c);
        if (SamePositiveSign(ac, bc))
            return Norm(a);
        if (SameNegativeSign(ac, bc))
            return Norm(b);

        return std::fabs(Cross(a, c)) / cNorm;
    }

    namespace point_to_boundary_distance_detail
    {

        inline double PointToPolygonDistance(
            const StPoint &point,
            const StPolygon &polygon)
        {
            if (!IsFinite(point) || !PolygonIsFinite(polygon))
                return std::numeric_limits<double>::infinity();

            const double edge12 = PointToLineSegmentDistance(point, polygon.p1, polygon.p2);
            const double edge13 = PointToLineSegmentDistance(point, polygon.p1, polygon.p3);
            const double edge34 = PointToLineSegmentDistance(point, polygon.p3, polygon.p4);
            const double edge24 = PointToLineSegmentDistance(point, polygon.p2, polygon.p4);
            const double minDistance =
                std::min(std::min(edge12, edge13), std::min(edge34, edge24));

            if (minDistance <= kDistanceEpsilon)
                return 0.0;

            const StPoint vertices[] = {
                polygon.p1,
                polygon.p3,
                polygon.p4,
                polygon.p2,
            };
            if (IsInsideConvexPolygon(point, vertices, 4))
            {
                return -1.0;
            }

            return minDistance;
        }

    } // namespace point_to_boundary_distance_detail

    template <typename CutInAndOutInfoT>
    bool ComputePointToBoundaryDistances(
        const std::vector<CutInAndOutInfoT> &cutInAndOutInfos,
        const StPoint &startPoint,
        std::vector<PointToBoundaryDistanceResult> *result)
    {
        if (result == nullptr)
            return false;

        result->clear();
        result->reserve(cutInAndOutInfos.size());
        for (const CutInAndOutInfoT &info : cutInAndOutInfos)
        {
            const point_to_boundary_distance_detail::StPolygon polygon =
                point_to_boundary_distance_detail::MakePolygon(
                    info.tin,
                    info.tout,
                    info.sinmin,
                    info.sinmax,
                    info.soutmin,
                    info.soutmax);

            result->push_back(
                {info.id,
                 point_to_boundary_distance_detail::PointToPolygonDistance(
                     startPoint,
                     polygon)});
        }

        return true;
    }

} // namespace rsim_driver
