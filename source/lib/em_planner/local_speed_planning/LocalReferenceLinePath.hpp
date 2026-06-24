#pragma once

#include "local_path_planning/quadratic_programming/QpPathOptimizer.hpp"

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
    double theta = 0.0;
    double s = 0.0;
};

using localreferencelinepath = std::vector<localreferencelinepoint>;

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
