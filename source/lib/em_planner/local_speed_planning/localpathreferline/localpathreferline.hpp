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
};

using localreferencelinepath = std::vector<localreferencelinepoint>;

namespace local_reference_line_detail
{

constexpr double kDkSIntervalEpsilon = 1e-6;


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
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.heading) || !std::isfinite(point.kappa))
        {
            result->clear();
            return false;
        }
        if (i > 0)
        {
            const CartesianPathPoint& previous = localcartesianpath[i - 1];
            const double segment_length =
                std::hypot(point.x - previous.x, point.y - previous.y);
            if (!std::isfinite(segment_length) || segment_length <= 1e-9)
            {
                result->clear();
                return false;
            }
            accumulatedS += segment_length;
        }

        result->push_back({point.x,
                           point.y,
                           point.kappa,
                           point.heading,
                           accumulatedS});
    }

    return true;
}



}  // namespace rsim_driver
