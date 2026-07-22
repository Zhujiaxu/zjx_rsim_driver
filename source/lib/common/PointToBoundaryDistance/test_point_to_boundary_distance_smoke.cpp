#include "PointToBoundaryDistance.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

struct Boundary
{
    int id;
    double tin;
    double tout;
    double sinmin;
    double sinmax;
    double soutmin;
    double soutmax;
};

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

}  // namespace

int main()
{
    if (!Require(std::fabs(rsim_driver::PointToLineSegmentDistance(
                              {1.0, 1.0}, {0.0, 0.0}, {2.0, 0.0}) -
                          1.0) < 1e-9,
                 "point-to-segment distance should be Euclidean"))
        return 1;

    const std::vector<Boundary> boundaries{{7, 0.0, 2.0, 4.0, 6.0, 4.0, 6.0}};
    std::vector<rsim_driver::PointToBoundaryDistanceResult> distances;
    if (!Require(rsim_driver::ComputePointToBoundaryDistances(
                     boundaries, {5.0, 1.0}, &distances) &&
                     distances.size() == 1 && distances[0].id == 7 &&
                     distances[0].distance == 0.0,
                 "point inside ST polygon should have zero boundary distance"))
        return 1;
    if (!Require(!rsim_driver::ComputePointToBoundaryDistances(
                     boundaries, {5.0, 1.0}, nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS point_to_boundary_distance smoke\n");
    return 0;
}
