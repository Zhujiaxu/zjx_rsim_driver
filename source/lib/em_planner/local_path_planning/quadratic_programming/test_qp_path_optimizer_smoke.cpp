#include "QpPathOptimizer.hpp"

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

std::vector<rsim_driver::DpPathPoint> MakePath(double l = 0.0,
                                               int count = 5)
{
    std::vector<rsim_driver::DpPathPoint> path;
    path.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
        path.push_back({static_cast<double>(i), l, 0.0, 0.0});
    return path;
}

std::vector<RefPoint> MakeReference(int count = 5)
{
    std::vector<RefPoint> reference;
    reference.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        const double s = static_cast<double>(i);
        reference.push_back({s, 0.0, 0.0, 0.0, 0.0, s});
    }
    return reference;
}

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

rsim_driver::StaticFrenetObstacle MakeObstacle(double s,
                                               double l,
                                               double length,
                                               double width)
{
    rsim_driver::StaticFrenetObstacle obstacle;
    obstacle.id = 1;
    obstacle.s = s;
    obstacle.l = l;
    obstacle.length = length;
    obstacle.width = width;
    return obstacle;
}

bool CheckBounds(const std::vector<rsim_driver::DpPathPoint>& path,
                 const rsim_driver::DrivableArea& area)
{
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i].l < area.right_boundary[i].l - 1e-4 ||
            path[i].l > area.left_boundary[i].l + 1e-4)
        {
            return false;
        }
    }
    return true;
}

bool CheckContinuity(const std::vector<rsim_driver::DpPathPoint>& path,
                     double tolerance = 2e-4)
{
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const double delta = path[i].s - path[i - 1].s;
        const double deltas = delta;
        const double deltas2 = deltas * deltas;
        const double deltas3 = deltas2 * deltas;
        const double positionResidual =
            path[i].l - path[i - 1].l -
            path[i].l_prime * deltas +
            path[i - 1].l_double_prime * deltas3 / (6.0 * delta) +
            path[i].l_double_prime *
                (deltas2 / 2.0 - deltas3 / (6.0 * delta));
        const double derivativeResidual =
            path[i].l_prime - path[i - 1].l_prime -
            0.5 * (path[i - 1].l_double_prime +
                   path[i].l_double_prime) *
                delta;

        if (std::fabs(positionResidual) > tolerance ||
            std::fabs(derivativeResidual) > tolerance)
        {
            return false;
        }
    }
    return true;
}

bool CheckJerkResidualIsSmall(const std::vector<rsim_driver::DpPathPoint>& path,
                              double tolerance = 2e-3)
{
    for (std::size_t i = 0; i + 1 < path.size(); ++i)
    {
        const double ds = path[i + 1].s - path[i].s;
        const double residual =
            path[i + 1].l_double_prime + path[i].l_prime / ds;
        if (std::fabs(residual) > tolerance)
            return false;
    }
    return true;
}

}  // namespace

int main()
{
    rsim_driver::QpPathOptimizerConfig config;
    config.weight_reference_l = 10.0;
    config.weight_smooth_l_prime = 1.0;
    config.weight_smooth_l_double_prime = 1.0;
    config.weight_jerk = 1.0;
    config.weight_collision = 10.0;
    config.weight_drivable_area_center = 1.0;

    rsim_driver::QpPathOptimizer optimizer(config);
    rsim_driver::QpPathResult result;
    std::vector<rsim_driver::DpPathPoint> localfrenetpath;

    const std::vector<rsim_driver::DpPathPoint> path = MakePath();
    const std::vector<RefPoint> reference = MakeReference();
    const rsim_driver::DrivableArea symmetricArea = MakeArea(path, 3.5, -3.5);
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
                     localfrenetpath.size() == path.size() &&
                     result.localcartesianpath.size() == path.size(),
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
    if (!Require(CheckBounds(localfrenetpath, symmetricArea),
                 "QP output should stay inside drivable area bounds"))
        return 1;
    if (!Require(CheckContinuity(localfrenetpath),
                 "QP output should satisfy second-order continuity constraints"))
        return 1;
    if (!Require(CheckJerkResidualIsSmall(localfrenetpath),
                 "QP output should satisfy requested jerk residual in simple case"))
        return 1;

    const std::vector<rsim_driver::StaticFrenetObstacle> farLateralObstacles = {
        MakeObstacle(2.0, 20.0, 4.0, 2.0),
    };
    if (!Require(optimizer.Optimize(MakeStart(),
                                    path,
                                    symmetricArea,
                                    farLateralObstacles,
                                    reference,
                                    &localfrenetpath,
                                    &result),
                 "QP optimizer should solve far-lateral obstacle scene"))
        return 1;
    for (const rsim_driver::DpPathPoint& point : localfrenetpath)
    {
        if (!Require(Near(point.l, 0.0, 2e-4),
                     "far-lateral obstacle should not pull path toward boundary"))
            return 1;
    }

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
        if (!Require(Near(point.l, 1.0, 3e-4),
                     "high center weight should keep path near drivable area center"))
            return 1;
    }

    optimizer.SetConfig(config);
    const std::vector<rsim_driver::DpPathPoint> leftPath = MakePath(1.2);
    rsim_driver::DrivableArea narrowedArea = MakeArea(leftPath, 3.5, 1.0);
    const std::vector<rsim_driver::StaticFrenetObstacle> obstacles = {
        MakeObstacle(2.0, 0.0, 4.0, 2.0),
    };
    if (!Require(optimizer.Optimize(MakeStart(1.2),
                                    leftPath,
                                    narrowedArea,
                                    obstacles,
                                    reference,
                                    &localfrenetpath,
                                    &result),
                 "QP optimizer should solve obstacle narrowed drivable area"))
        return 1;
    if (!Require(CheckBounds(localfrenetpath, narrowedArea),
                 "QP output with obstacle should stay in narrowed drivable area"))
        return 1;
    if (!Require(CheckContinuity(localfrenetpath),
                 "QP output with obstacle should remain second-order continuous"))
        return 1;

    std::fprintf(stderr, "PASS qp_path_optimizer smoke\n");
    return 0;
}
