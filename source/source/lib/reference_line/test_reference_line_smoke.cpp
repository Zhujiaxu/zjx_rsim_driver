#include "ReferenceLineGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

std::vector<rsim_driver::WorldPoint> BuildStraightPath()
{
    std::vector<rsim_driver::WorldPoint> path;
    path.reserve(60);
    for (int i = 0; i < 60; ++i)
    {
        rsim_driver::WorldPoint point;
        point.x = static_cast<double>(i);
        point.y = 0.0;
        path.push_back(point);
    }
    return path;
}

std::vector<rsim_driver::WorldPoint> BuildStraightPath(int count)
{
    std::vector<rsim_driver::WorldPoint> path;
    path.reserve(static_cast<std::size_t>(std::max(0, count)));
    for (int i = 0; i < count; ++i)
    {
        rsim_driver::WorldPoint point;
        point.x = static_cast<double>(i);
        point.y = 0.0;
        path.push_back(point);
    }
    return path;
}

}  // namespace

int main()
{
    std::vector<rsim_driver::WorldPoint> path = BuildStraightPath();

    rsim_driver::ReferenceLineGenerator::GenerateConfig config;
    config.forwardPoints = 10;
    config.backwardPoints = 5;
    config.smoother.coordinateBound = 1e-6;

    rsim_driver::ReferenceLineGenerator generator(config);
    auto referenceLine = generator.Generate(path, 20.2, 3.0);
    if (!Require(referenceLine != nullptr, "reference line should be generated"))
        return 1;
    if (!Require(referenceLine->points.size() == 16,
                 "reference line should contain configured window points"))
        return 1;
    if (!Require(generator.lastMatchPointIndex() == 20,
                 "global match point should be nearest global path point"))
        return 1;

    const rsim_driver::ReferencePoint projection = generator.lastProjectionPoint();
    if (!Require(std::fabs(projection.x - 20.2) < 1e-4 &&
                 std::fabs(projection.y) < 1e-4,
                 "projection point should lie on the straight reference line"))
        return 1;
    if (!Require(std::fabs(projection.s) < 1e-9,
                 "projection point should be stored with s = 0"))
        return 1;

    if (!Require(referenceLine->points.front().s < 0.0,
                 "reference points behind projection should have negative s"))
        return 1;
    if (!Require(referenceLine->points.back().s > 0.0,
                 "reference points ahead of projection should have positive s"))
        return 1;

    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    if (!Require(referenceLine->Eval(0.0, 0.0, &x, &y, &heading),
                 "reference line Eval at projection s should succeed"))
        return 1;
    if (!Require(std::fabs(x - projection.x) < 1e-4 &&
                 std::fabs(y - projection.y) < 1e-4,
                 "Eval(0, 0) should return the projection point"))
        return 1;

    const std::size_t firstMatch = generator.lastMatchPointIndex();
    referenceLine = generator.Generate(path, 25.3, -2.0);
    if (!Require(referenceLine != nullptr, "second reference line should be generated"))
        return 1;
    if (!Require(generator.lastMatchPointIndex() >= firstMatch,
                 "global match point should advance monotonically"))
        return 1;
    if (!Require(generator.lastMatchPointIndex() == 25,
                 "second global match point should be nearest global path point"))
        return 1;

    rsim_driver::ReferenceLineGenerator startGenerator(config);
    referenceLine = startGenerator.Generate(path, 0.0, 0.0);
    if (!Require(referenceLine != nullptr, "start reference line should be generated"))
        return 1;
    if (!Require(referenceLine->points.size() == 16,
                 "start reference line should be padded from forward points"))
        return 1;
    if (!Require(std::fabs(referenceLine->points.front().x - 0.0) < 1e-4 &&
                 std::fabs(referenceLine->points.back().x - 15.0) < 1e-4,
                 "start reference line should cover global window [0, 15]"))
        return 1;

    rsim_driver::ReferenceLineGenerator endGenerator(config);
    referenceLine = endGenerator.Generate(path, 59.0, 0.0);
    if (!Require(referenceLine != nullptr, "end reference line should be generated"))
        return 1;
    if (!Require(referenceLine->points.size() == 16,
                 "end reference line should be padded from backward points"))
        return 1;
    if (!Require(std::fabs(referenceLine->points.front().x - 44.0) < 1e-4 &&
                 std::fabs(referenceLine->points.back().x - 59.0) < 1e-4,
                 "end reference line should cover global window [44, 59]"))
        return 1;

    std::vector<rsim_driver::WorldPoint> shortPath = BuildStraightPath(12);
    rsim_driver::ReferenceLineGenerator shortGenerator(config);
    referenceLine = shortGenerator.Generate(shortPath, 6.0, 0.0);
    if (!Require(referenceLine != nullptr, "short reference line should be generated"))
        return 1;
    if (!Require(referenceLine->points.size() == shortPath.size(),
                 "short reference line should contain all available points"))
        return 1;
    if (!Require(std::fabs(referenceLine->points.front().x - 0.0) < 1e-4 &&
                 std::fabs(referenceLine->points.back().x - 11.0) < 1e-4,
                 "short reference line should cover the full global path"))
        return 1;

    std::fprintf(stderr, "PASS reference_line smoke\n");
    return 0;
}
