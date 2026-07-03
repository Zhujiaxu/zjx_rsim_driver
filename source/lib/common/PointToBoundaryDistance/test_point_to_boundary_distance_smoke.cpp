#include "PointToBoundaryDistance.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{

struct CutInAndOutInfo
{
    int32_t id = 0;
    double tin = 0.0;
    double tout = 0.0;
    double sin = 0.0;
    double sout = 0.0;
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
    const rsim_driver::StPoint start{5.0, 3.0};

    if (!Require(Near(rsim_driver::PointToLineSegmentDistance(
                          start,
                          {0.0, 0.0},
                          {10.0, 0.0}),
                      3.0),
                 "point projection inside segment should use perpendicular distance"))
        return 1;

    if (!Require(Near(rsim_driver::PointToLineSegmentDistance(
                          {-1.0, 0.0},
                          {0.0, 0.0},
                          {10.0, 0.0}),
                      1.0),
                 "point before segment start should use distance to start endpoint"))
        return 1;

    if (!Require(Near(rsim_driver::PointToLineSegmentDistance(
                          {11.0, 0.0},
                          {0.0, 0.0},
                          {10.0, 0.0}),
                      1.0),
                 "point after segment end should use distance to end endpoint"))
        return 1;

    if (!Require(Near(rsim_driver::PointToLineSegmentDistance(
                          {4.0, 3.0},
                          {1.0, 1.0},
                          {1.0, 1.0}),
                      std::sqrt(13.0)),
                 "zero-length segment should use distance to the endpoint"))
        return 1;

    if (!Require(std::isinf(rsim_driver::PointToLineSegmentDistance(
                     start,
                     {0.0, 0.0},
                     {std::numeric_limits<double>::infinity(), 0.0})),
                 "non-finite segment endpoint should return infinity"))
        return 1;

    const std::vector<CutInAndOutInfo> cutInAndOutInfos = {
        {101, 0.0, 0.0, 0.0, 10.0},
        {102, 0.0, 0.0, 0.0, 0.0},
    };
    std::vector<rsim_driver::PointToLineDistanceResult> distances;
    if (!Require(rsim_driver::ComputePointToCutInAndOutLineDistances(
                     cutInAndOutInfos,
                     start,
                     &distances),
                 "cut in and out line distance computation should succeed"))
        return 1;

    if (!Require(distances.size() == 2 &&
                     distances[0].id == 101 &&
                     Near(distances[0].distance, 3.0) &&
                     distances[1].id == 102 &&
                     Near(distances[1].distance, std::sqrt(34.0)),
                 "line distance result should keep obstacle ids and distances"))
        return 1;

    if (!Require(!rsim_driver::ComputePointToCutInAndOutLineDistances(
                     cutInAndOutInfos,
                     start,
                     nullptr),
                 "null line distance output should fail"))
        return 1;

    std::fprintf(stderr, "PASS point_to_boundary_distance smoke\n");
    return 0;
}
