#include "QpPathOptimizer.hpp"

#include <cmath>
#include <cstdio>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

rsim_driver::DrivableArea MakeArea(int count)
{
    rsim_driver::DrivableArea area;
    for (int i = 0; i < count; ++i)
    {
        const double s = static_cast<double>(i);
        area.left_boundary.push_back({s, 4.0});
        area.right_boundary.push_back({s, -4.0});
    }
    return area;
}

}  // namespace

int main()
{
    rsim_driver::QpPathOptimizerConfig config;
    config.num_points = 5;
    config.ds = 1.0;
    rsim_driver::QpPathOptimizer optimizer(config);
    rsim_driver::QpPathResult result;
    rsim_driver::CartesianFrenetState start;

    if (!Require(optimizer.Optimize(start, MakeArea(5), &result) &&
                     result.qpsuccess && result.localfrenetpath.size() == 5,
                 "valid symmetric corridor should solve"))
        return 1;
    for (const auto &point : result.localfrenetpath)
    {
        if (!Require(std::isfinite(point.l) &&
                         std::isfinite(point.l_prime) &&
                         std::isfinite(point.l_double_prime),
                     "path QP solution should be finite"))
            return 1;
    }

    rsim_driver::DrivableArea mismatched = MakeArea(5);
    mismatched.right_boundary.pop_back();
    if (!Require(!optimizer.Optimize(start, mismatched, &result) &&
                     !result.qpsuccess && result.localfrenetpath.empty(),
                 "mismatched boundary sizes should fail without stale output"))
        return 1;

    rsim_driver::DrivableArea unsorted = MakeArea(5);
    unsorted.left_boundary[2].s = unsorted.left_boundary[1].s;
    if (!Require(!optimizer.Optimize(start, unsorted,&result),
                 "non-increasing boundary grid should fail"))
        return 1;
    if (!Require(!optimizer.Optimize(start, MakeArea(5), nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS qp_path_optimizer smoke\n");
    return 0;
}
