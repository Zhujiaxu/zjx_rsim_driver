#pragma once

#include "QpPathOptimizer.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

struct localreferencelinepoint
{
    double x = 0.0;
    double y = 0.0;
    double k = 0.0;
    double hdg = 0.0;
    double s = 0.0;
    double dk = 0.0;
};

using localreferencelinepath = std::vector<localreferencelinepoint>;

namespace local_reference_line_detail
{

constexpr double kDkSIntervalEpsilon = 1e-6;

inline double CalculateDkBetween(const localreferencelinepoint& previous,
                                 const localreferencelinepoint& next)
{
    const double ds = next.s - previous.s;
    return std::fabs(ds) > kDkSIntervalEpsilon
               ? (next.k - previous.k) / ds
               : 0.0;
}

}  // namespace local_reference_line_detail

/*inline bool CalculateLocalReferenceLineDk(localreferencelinepath* path)
{
    if (path == nullptr)
        return false;

    if (path->empty())
        return true;

    if (path->size() == 1)
    {
        path->front().dk = 0.0;
        return true;
    }

    for (std::size_t i = 0; i < path->size(); ++i)
    {
        if (i == 0)
        {
            (*path)[i].dk =
                local_reference_line_detail::CalculateDkBetween((*path)[0],
                                                                 (*path)[1]);
        }
        else if (i + 1 == path->size())
        {
            (*path)[i].dk =
                local_reference_line_detail::CalculateDkBetween((*path)[i - 1],
                                                                 (*path)[i]);
        }
        else
        {
            (*path)[i].dk =
                local_reference_line_detail::CalculateDkBetween((*path)[i-1],
                                                                 (*path)[i + 1]);
        }
    }

    return true;
}*/

inline bool LocalCartesianPathToReferenceLinePath(
    const std::vector<CartesianPathPoint>& localcartesianpath,
    localreferencelinepath* result)
{
    if (result == nullptr)
        return false;

    result->clear();
    if (localcartesianpath.empty())
        return false;

    result->reserve(localcartesianpath.size());
    double accumulatedS = 0.0;
    for (std::size_t i = 0; i < localcartesianpath.size(); ++i)
    {
        const CartesianPathPoint& point = localcartesianpath[i];
        if (i > 0)
        {
            const CartesianPathPoint& previous = localcartesianpath[i - 1];
            accumulatedS += std::hypot(point.x - previous.x,
                                       point.y - previous.y);
        }

        result->push_back({point.x,
                           point.y,
                           point.kappa,
                           point.heading,
                           accumulatedS});
    }

    // CalculateLocalReferenceLineDk(result);
    return true;
}

inline bool QpPathResultToLocalReferenceLinePath(
    const QpPathResult& qpPathResult,
    localreferencelinepath* result)
{
    return LocalCartesianPathToReferenceLinePath(qpPathResult.localcartesianpath,
                                                result);
}

}  // namespace rsim_driver
