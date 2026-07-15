#include "QpPathOptimizer.hpp"
#include "qpincreasepoints.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

struct RefPoint
{
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
    double k = 0.0;
    double dk = 0.0;
    double s = 0.0;
};

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-4)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::CartesianFrenetState MakeStart(double l = 0.0,
                                            double lPrime = 0.0,
                                            double lDoublePrime = 0.0)
{
    rsim_driver::CartesianFrenetState start;
    start.s = 0.0;
    start.l = l;
    start.l_prime = lPrime;
    start.l_double_prime = lDoublePrime;
    return start;
}

// Build a DP coarse path for ValidInput; points at uniform s
std::vector<rsim_driver::DpPathPoint> MakeCoarsePath(int count,
                                                     double step,
                                                     double l = 0.0)
{
    std::vector<rsim_driver::DpPathPoint> path;
    path.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
        path.push_back({static_cast<double>(i) * step, l, 0.0, 0.0});
    return path;
}

std::vector<RefPoint> MakeReference(int count = 81)
{
    std::vector<RefPoint> reference;
    reference.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const double s = static_cast<double>(i) * 0.5;
        reference.push_back({s, 0.0, 0.0, 0.0, 0.0, s});
    }
    return reference;
}

// Build DrivableArea on the same s as coarsePath
rsim_driver::DrivableArea MakeArea(
    const std::vector<rsim_driver::DpPathPoint>& path,
    double left,
    double right)
{
    rsim_driver::DrivableArea area;
    area.left_boundary.reserve(path.size());
    area.right_boundary.reserve(path.size());
    for (const rsim_driver::DpPathPoint& point : path)
    {
        area.left_boundary.push_back({point.s, left});
        area.right_boundary.push_back({point.s, right});
    }
    return area;
}

// Check that optimized path satisfies both third-order Taylor constraints.
bool CheckTaylor(const std::vector<rsim_driver::DpPathPoint>& path,
                 double ds,
                 double tolerance = 2e-4)
{
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const double residual =
            path[i].l - path[i - 1].l -
            ds * path[i - 1].l_prime -
            (ds * ds / 3.0) * path[i - 1].l_double_prime -
            (ds * ds / 6.0) * path[i].l_double_prime;
        if (std::fabs(residual) > tolerance)
            return false;

        const double lPrimeResidual =
            path[i].l_prime - path[i - 1].l_prime -
            0.5 * ds * path[i - 1].l_double_prime -
            0.5 * ds * path[i].l_double_prime;
        if (std::fabs(lPrimeResidual) > tolerance)
            return false;
    }
    return true;
}

// Check that optimized path satisfies corner constraints for i >= 1
bool CheckCorners(const std::vector<rsim_driver::DpPathPoint>& path,
                  const rsim_driver::DrivableArea& area,
                  double halfLen,
                  double halfWid,
                  double tolerance = 1e-4)
{
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const double si = path[i].s;
        const double l = path[i].l;
        const double lp = path[i].l_prime;

        // Interpolate area at front and rear axle positions
        // (reuse InterpolateBoundary logic inline for test independence)
        auto interpolate = [&area](double s) -> rsim_driver::SlPoint {
            const auto& b = area.left_boundary;
            if (s <= b.front().s) return {s, b.front().l};
            if (s >= b.back().s) return {s, b.back().l};
            for (std::size_t j = 1; j < b.size(); ++j) {
                if (s <= b[j].s) {
                    double d = b[j].s - b[j-1].s;
                    double r = (d > 1e-9) ? std::clamp((s - b[j-1].s) / d, 0.0, 1.0) : 0.0;
                    return {s, b[j-1].l + (b[j].l - b[j-1].l) * r};
                }
            }
            return {s, b.back().l};
        };

        auto interpR = [&area](double s) -> rsim_driver::SlPoint {
            const auto& b = area.right_boundary;
            if (s <= b.front().s) return {s, b.front().l};
            if (s >= b.back().s) return {s, b.back().l};
            for (std::size_t j = 1; j < b.size(); ++j) {
                if (s <= b[j].s) {
                    double d = b[j].s - b[j-1].s;
                    double r = (d > 1e-9) ? std::clamp((s - b[j-1].s) / d, 0.0, 1.0) : 0.0;
                    return {s, b[j-1].l + (b[j].l - b[j-1].l) * r};
                }
            }
            return {s, b.back().l};
        };

        const double lf = interpolate(si + halfLen).l;
        const double rf = interpR(si + halfLen).l;
        const double lr_bound = interpolate(si - halfLen).l;
        const double rr_bound = interpR(si - halfLen).l;

        const double front = l + halfLen * lp;
        const double rear = l - halfLen * lp;

        if (front < rf + halfWid - tolerance ||
            front > lf - halfWid + tolerance)
            return false;
        if (rear < rr_bound + halfWid - tolerance ||
            rear > lr_bound - halfWid + tolerance)
            return false;
    }
    return true;
}

}  // namespace

int main()
{
    rsim_driver::QpPathOptimizerConfig config;
    config.num_points = 21;
    config.ds = 2.0;
    config.ego_length = 4.0;
    config.ego_width = 2.0;
    config.weight_reference_l = 10.0;
    config.weight_smooth_l_prime = 1.0;
    config.weight_smooth_l_double_prime = 1.0;
    config.weight_jerk = 1.0;
    config.weight_drivable_area_center = 1.0;

    const double halfLen = 0.5 * config.ego_length;
    const double halfWid = 0.5 * config.ego_width;

    rsim_driver::QpPathOptimizer optimizer(config);
    rsim_driver::QpPathResult result;
    std::vector<rsim_driver::DpPathPoint> localfrenetpath;

    // coarse path must cover the uniform s grid range:
    // start.s=0, end=0+(21-1)*2=40m. Use 81 pts at 0.5m = 40m.
    const int coarseCount = 81;
    const double coarseDs = 0.5;
    const std::vector<rsim_driver::DpPathPoint> path =
        MakeCoarsePath(coarseCount, coarseDs);
    const std::vector<RefPoint> reference = MakeReference();
    const rsim_driver::DrivableArea symmetricArea = MakeArea(path, 3.5, -3.5);

    // --- Test 1: symmetric empty scene ---
    if (!Require(optimizer.Optimize(MakeStart(),
                                    path,
                                    symmetricArea,
                                    {},
                                    reference,
                                    &localfrenetpath,
                                    &result),
                 "QP optimizer should solve symmetric empty scene"))
        return 1;
    if (!Require(result.qpsuccess &&
                     localfrenetpath.size() == static_cast<std::size_t>(config.num_points) &&
                     result.localcartesianpath.size() == static_cast<std::size_t>(config.num_points),
                 "QP result should report success with matching path sizes"))
        return 1;
    if (!Require(Near(localfrenetpath.front().l, 0.0) &&
                     Near(localfrenetpath.front().l_prime, 0.0) &&
                     Near(localfrenetpath.front().l_double_prime, 0.0),
                 "QP result should satisfy start hard constraints"))
        return 1;
    for (const rsim_driver::DpPathPoint& point : localfrenetpath)
    {
        if (!Require(Near(point.l, 0.0, 2e-4),
                     "symmetric empty scene should stay close to reference line"))
            return 1;
    }
    // Verify uniform s grid
    for (std::size_t i = 0; i < localfrenetpath.size(); ++i)
    {
        const double expectedS = static_cast<double>(i) * config.ds;
        if (!Require(Near(localfrenetpath[i].s, expectedS),
                     "QP output should have uniform s spacing"))
            return 1;
    }
    // Verify Taylor constraints
    if (!Require(CheckTaylor(localfrenetpath, config.ds),
                 "QP output should satisfy Taylor expansion constraints"))
        return 1;

    rsim_driver::QpIncreasePoints densifier;
    std::vector<rsim_driver::DpPathPoint> densifiedPath;
    if (!Require(densifier.increasepoints(localfrenetpath, &densifiedPath),
                 "QP output should be Taylor-consistent for densification"))
        return 1;
    if (!Require(densifiedPath.size() ==
                     1U + (localfrenetpath.size() - 1U) * 10U,
                 "QP densification should add nine interior points per segment"))
        return 1;
    for (std::size_t i = 0; i < densifiedPath.size(); ++i)
    {
        const double expectedS = static_cast<double>(i) * config.ds / 10.0;
        if (!Require(Near(densifiedPath[i].s, expectedS),
                     "QP densification should use uniform subsegment spacing"))
            return 1;
    }
    std::vector<rsim_driver::CartesianPathPoint> densifiedCartesianPath;
    if (!Require(rsim_driver::FrenetPathToCartesian(reference,
                                                     densifiedPath,
                                                     &densifiedCartesianPath),
                 "densified Frenet path should convert back to Cartesian"))
        return 1;
    if (!Require(densifiedCartesianPath.size() == densifiedPath.size(),
                 "densified Cartesian path should retain all Frenet samples"))
        return 1;

    // Verify corner constraints
    if (!Require(CheckCorners(localfrenetpath, symmetricArea, halfLen, halfWid),
                 "QP output should satisfy corner hard constraints"))
        return 1;
    // Verify Cartesian conversion
    for (std::size_t i = 0; i < result.localcartesianpath.size(); ++i)
    {
        if (!Require(Near(result.localcartesianpath[i].x,
                          localfrenetpath[i].s,
                          1e-4) &&
                         Near(result.localcartesianpath[i].y,
                              localfrenetpath[i].l,
                              1e-4),
                     "Cartesian QP path should match straight reference conversion"))
            return 1;
    }

    // --- Test 2: drivable area center ---
    rsim_driver::QpPathOptimizerConfig centerConfig = config;
    centerConfig.weight_reference_l = 0.0;
    centerConfig.weight_drivable_area_center = 100.0;
    optimizer.SetConfig(centerConfig);
    const rsim_driver::DrivableArea shiftedArea = MakeArea(path, 2.0, 0.0);
    if (!Require(optimizer.Optimize(MakeStart(1.0),
                                    path,
                                    shiftedArea,
                                    {},
                                    reference,
                                    &localfrenetpath,
                                    &result),
                 "QP optimizer should solve shifted drivable center scene"))
        return 1;
    for (const rsim_driver::DpPathPoint& point : localfrenetpath)
    {
        if (!Require(Near(point.l, 1.0, 5e-3),
                     "high center weight should keep path near drivable area center"))
            return 1;
    }
    if (!Require(CheckCorners(localfrenetpath, shiftedArea, halfLen, halfWid),
                 "shifted scene should satisfy corner constraints"))
        return 1;
    if (!Require(CheckTaylor(localfrenetpath, config.ds),
                 "shifted scene should satisfy Taylor constraints"))
        return 1;

    // --- Test 3: path continuity with non-zero start ---
    optimizer.SetConfig(config);
    if (!Require(optimizer.Optimize(MakeStart(1.0, 0.0, 0.0),
                                    path,
                                    symmetricArea,
                                    {},
                                    reference,
                                    &localfrenetpath,
                                    &result),
                 "QP optimizer should solve with offset start"))
        return 1;
    if (!Require(CheckTaylor(localfrenetpath, config.ds),
                 "offset-start scene should satisfy Taylor constraints"))
        return 1;
    if (!Require(CheckCorners(localfrenetpath, symmetricArea, halfLen, halfWid),
                 "offset-start scene should satisfy corner constraints"))
        return 1;

    std::fprintf(stderr, "PASS qp_path_optimizer smoke\n");
    return 0;
}
