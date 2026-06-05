#include "PathMatcher.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

struct XYPoint
{
    double x = 0.0;
    double y = 0.0;
};

struct HeadingPoint
{
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
};

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

}  // namespace

int main()
{
    const std::vector<XYPoint> xyPoints = {
        {0.0, 0.0},
        {10.0, 0.0},
        {20.0, 0.0},
    };

    const std::size_t xyMatch =
        rsim_driver::FindMatchPointIndex(xyPoints, 12.0, 3.0);
    if (!Require(xyMatch == 1, "xy point match index should be nearest point"))
        return 1;

    const XYPoint xyProjection =
        rsim_driver::FindProjectionPoint(xyPoints, 12.0, 3.0);
    if (!Require(Near(xyProjection.x, 12.0) && Near(xyProjection.y, 0.0),
                 "xy point projection should lie on the straight path"))
        return 1;

    const std::vector<HeadingPoint> headingPoints = {
        {0.0, 0.0, 0.0},
        {10.0, 0.0, 1.0},
        {20.0, 0.0, 2.0},
    };
    const HeadingPoint headingProjection =
        rsim_driver::FindProjectionPoint(headingPoints, 12.0, 3.0);
    if (!Require(Near(headingProjection.x, 12.0) &&
                 Near(headingProjection.y, 0.0) &&
                 Near(headingProjection.hdg, 0.0),
                 "heading point projection should fill tangent heading"))
        return 1;

    const std::vector<XYPoint> empty;
    const XYPoint emptyProjection =
        rsim_driver::FindProjectionPoint(empty, 12.0, 3.0);
    if (!Require(rsim_driver::FindMatchPointIndex(empty, 12.0, 3.0) == 0,
                 "empty match index should be zero"))
        return 1;
    if (!Require(Near(emptyProjection.x, 0.0) && Near(emptyProjection.y, 0.0),
                 "empty projection should return default point"))
        return 1;

    std::fprintf(stderr, "PASS path_matcher smoke\n");
    return 0;
}
