#include "localpathreferline.hpp"

#include <cmath>
#include <cstdio>

namespace
{

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::CartesianPathPoint MakeCartesianPoint(double x,
                                                   double y,
                                                   double heading,
                                                   double kappa)
{
    rsim_driver::CartesianPathPoint point;
    point.x = x;
    point.y = y;
    point.heading = heading;
    point.kappa = kappa;
    return point;
}

}  // namespace

int main()
{
    rsim_driver::QpPathResult qpResult;
    qpResult.qpsuccess = true;
    qpResult.localcartesianpath = {
        MakeCartesianPoint(1.0, 2.0, 0.10, 0.01),
        MakeCartesianPoint(4.0, 6.0, 0.20, 0.02),
        MakeCartesianPoint(4.0, 10.0, 0.30, 0.03),
    };

    rsim_driver::localreferencelinepath path;
    if (!Require(rsim_driver::QpPathResultToLocalReferenceLinePath(qpResult,
                                                                    &path),
                 "QP path result should convert to local reference line path"))
        return 1;

    if (!Require(path.size() == qpResult.localcartesianpath.size(),
                 "converted path should keep the same number of points"))
        return 1;

    if (!Require(Near(path[0].x, 1.0) &&
                     Near(path[0].y, 2.0) &&
                     Near(path[0].theta, 0.10) &&
                     Near(path[0].k, 0.01) &&
                     Near(path[0].s, 0.0),
                 "first local reference line point should map fields directly"))
        return 1;

    if (!Require(Near(path[1].s, 5.0) &&
                     Near(path[2].s, 9.0),
                 "local reference line s should be cumulative Cartesian distance"))
        return 1;

    if (!Require(Near(path[0].dk, 0.002) &&
                     Near(path[1].dk, 2.0 / 900.0) &&
                     Near(path[2].dk, 0.0025),
                 "local reference line dk should use one-sided and centered differences"))
        return 1;

    rsim_driver::localreferencelinepath singlePointPath(1);
    singlePointPath.front().dk = 1.0;
    if (!Require(rsim_driver::CalculateLocalReferenceLineDk(&singlePointPath) &&
                     Near(singlePointPath.front().dk, 0.0),
                 "single-point local reference line dk should be zero"))
        return 1;

    rsim_driver::localreferencelinepath duplicateSPath(2);
    duplicateSPath[0].s = 1.0;
    duplicateSPath[0].k = 0.1;
    duplicateSPath[1].s = 1.0;
    duplicateSPath[1].k = 0.2;
    if (!Require(rsim_driver::CalculateLocalReferenceLineDk(&duplicateSPath) &&
                     Near(duplicateSPath[0].dk, 0.0) &&
                     Near(duplicateSPath[1].dk, 0.0),
                 "duplicate-s local reference line dk should be zero"))
        return 1;

    if (!Require(!rsim_driver::CalculateLocalReferenceLineDk(nullptr),
                 "null local reference line dk output should fail"))
        return 1;

    if (!Require(!rsim_driver::LocalCartesianPathToReferenceLinePath({}, &path) &&
                     path.empty(),
                 "empty Cartesian path should fail and clear output"))
        return 1;

    std::fprintf(stderr, "PASS local_reference_line_path smoke\n");
    return 0;
}
