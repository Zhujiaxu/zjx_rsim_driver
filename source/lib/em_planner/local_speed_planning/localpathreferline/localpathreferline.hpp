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



}  // namespace rsim_driver
