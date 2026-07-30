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

bool Near(double actual, double expected, double tolerance = 1e-6)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DrivableAreaResult MakeArea(int count)
{
    rsim_driver::DrivableAreaResult area;
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
    if (!Require(config.num_points == 113 && Near(config.ds, 0.25) &&
                     Near((config.num_points - 1) * config.ds, 28.0),
                 "default QP refinement should preserve the 28 m horizon"))
        return 1;
    config.num_points = 5;
    config.ds = 1.0;
    rsim_driver::QpPathOptimizer optimizer(config);
    rsim_driver::QpPathResult result;
    rsim_driver::StartPointFrenetState start;

    if (!Require(optimizer.Optimize(start, MakeArea(5), &result) &&
                     result.Flag ==
                         rsim_driver::QpPathOptimizerFallback::Success &&
                     result.localfrenetpath.size() == 5,
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

    rsim_driver::DrivableAreaResult mismatched = MakeArea(5);
    mismatched.right_boundary.pop_back();
    if (!Require(!optimizer.Optimize(start, mismatched, &result) &&
                     result.Flag ==
                         rsim_driver::QpPathOptimizerFallback::Other &&
                     result.localfrenetpath.empty(),
                 "mismatched boundary sizes should fail without stale output"))
        return 1;

    rsim_driver::DrivableAreaResult unsorted = MakeArea(5);
    unsorted.left_boundary[2].s = unsorted.left_boundary[1].s;
    if (!Require(!optimizer.Optimize(start, unsorted,&result),
                 "non-increasing boundary grid should fail"))
        return 1;
    if (!Require(!optimizer.Optimize(start, MakeArea(5), nullptr),
                 "null output should fail"))
        return 1;

    rsim_driver::QpPathOptimizerConfig envelopeConfig = config;
    envelopeConfig.num_points = 6;
    envelopeConfig.ego_length = 2.0;
    envelopeConfig.ego_width = 1.0;
    envelopeConfig.weight_reference_l = 0.0;
    envelopeConfig.weight_drivable_area_center = 10.0;
    rsim_driver::QpPathOptimizer envelopeOptimizer(envelopeConfig);
    rsim_driver::DrivableAreaResult discontinuous = MakeArea(6);
    for (std::size_t i = 3; i < discontinuous.right_boundary.size(); ++i)
        discontinuous.right_boundary[i].l = 1.0;
    if (!Require(envelopeOptimizer.Optimize(start, discontinuous, &result),
                 "discontinuous obstacle boundary should remain feasible") ||
        !Require(result.localfrenetpath.size() == 6 &&
                     result.localfrenetpath[2].l >= 1.5 - 1e-6 &&
                     Near(result.localfrenetpath[2].s, 2.0),
                 "full longitudinal envelope should see an interior boundary"))
    {
        return 1;
    }

    std::fprintf(stderr, "PASS qp_path_optimizer smoke\n");
    return 0;
}
