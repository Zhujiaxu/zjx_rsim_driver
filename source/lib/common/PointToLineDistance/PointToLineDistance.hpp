#pragma once

#include <cmath>
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

    struct PointToLineDistanceResult
    {
        int32_t id = 0;
        double distance = 0.0;
    };

    namespace point_to_line_distance_detail
    {

        constexpr double kDistanceEpsilon = 1e-9;

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

    } // namespace point_to_line_distance_detail

    inline double PointToLineSegmentDistance(const StPoint &point,
                                             const StPoint &segmentStart,
                                             const StPoint &segmentEnd)
    {
        using namespace point_to_line_distance_detail;

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

    template <typename CutInAndOutInfoT>
    bool ComputePointToCutInAndOutLineDistances(
        const std::vector<CutInAndOutInfoT> &cutInAndOutInfos,
        const StPoint &startPoint,
        std::vector<PointToLineDistanceResult> *result)
    {
        if (result == nullptr)
            return false;

        result->clear();
        result->reserve(cutInAndOutInfos.size());
        for (const CutInAndOutInfoT &info : cutInAndOutInfos)
        {
            const StPoint segmentStart{info.sin, info.tin};
            const StPoint segmentEnd{info.sout, info.tout};
            result->push_back({info.id,
                               PointToLineSegmentDistance(startPoint,
                                                          segmentStart,
                                                          segmentEnd)});
        }

        return true;
    }

} // namespace rsim_driver
