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
    double sinmin = 0.0;
    double sinmax = 0.0;
    double soutmin = 0.0;
    double soutmax = 0.0;
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

double PolygonDistance(const CutInAndOutInfo& info,
                       const rsim_driver::StPoint& point)
{
    std::vector<rsim_driver::PointToLineDistanceResult> distances;
    const std::vector<CutInAndOutInfo> infos = {info};
    if (!rsim_driver::ComputePointToCutInAndOutLineDistances(infos,
                                                             point,
                                                             &distances) ||
        distances.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    return distances.front().distance;
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

    const CutInAndOutInfo rectangle{101, 1.0, 3.0, 10.0, 20.0, 10.0, 20.0};
    if (!Require(Near(PolygonDistance(rectangle, {15.0, 2.0}), 0.0),
                 "point inside cut in and out polygon should return zero"))
        return 1;

    if (!Require(Near(PolygonDistance(rectangle, {10.0, 2.0}), 0.0),
                 "point on cut in and out polygon should return zero"))
        return 1;

    if (!Require(Near(PolygonDistance(rectangle, {15.0, 0.0}), 1.0),
                 "point before entry boundary should use p1 to p2 distance"))
        return 1;

    if (!Require(Near(PolygonDistance(rectangle, {8.0, 2.0}), 2.0),
                 "point below boundary should use p1 to p3 distance"))
        return 1;

    if (!Require(Near(PolygonDistance(rectangle, {15.0, 4.0}), 1.0),
                 "point after exit boundary should use p3 to p4 distance"))
        return 1;

    if (!Require(Near(PolygonDistance(rectangle, {23.0, 2.0}), 3.0),
                 "point above boundary should use p2 to p4 distance"))
        return 1;

    const CutInAndOutInfo nonFinite{102,
                                    1.0,
                                    std::numeric_limits<double>::infinity(),
                                    10.0,
                                    20.0,
                                    10.0,
                                    20.0};
    if (!Require(std::isinf(PolygonDistance(nonFinite, {15.0, 2.0})),
                 "non-finite cut in and out polygon should return infinity"))
        return 1;

    const std::vector<CutInAndOutInfo> cutInAndOutInfos = {
        rectangle,
        {103, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0},
    };
    std::vector<rsim_driver::PointToLineDistanceResult> distances;
    if (!Require(rsim_driver::ComputePointToCutInAndOutLineDistances(
                     cutInAndOutInfos,
                     {4.0, 1.0},
                     &distances),
                 "cut in and out line distance computation should succeed"))
        return 1;

    if (!Require(distances.size() == 2 &&
                     distances[0].id == 101 &&
                     Near(distances[0].distance, 6.0) &&
                     distances[1].id == 103 &&
                     Near(distances[1].distance, 2.0),
                 "line distance result should keep obstacle ids and polygon distances"))
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
