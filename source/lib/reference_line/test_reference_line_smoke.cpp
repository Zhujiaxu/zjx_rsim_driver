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

std::vector<rsim_driver::WorldPoint> BuildPath()
{
    std::vector<rsim_driver::WorldPoint> path;
    path.reserve(260);
    for (int i = 0; i < 260; ++i)
    {
        rsim_driver::WorldPoint point;
        point.x = static_cast<double>(i);
        point.y = 2.0 * std::sin(static_cast<double>(i) * 0.04);
        path.push_back(point);
    }
    return path;
}

std::size_t ExpectedWindowStart(const rsim_driver::ReferenceLineGenerator& generator)
{
    const std::size_t projection = generator.lastProjectionIndex();
    const std::size_t backward =
        static_cast<std::size_t>(std::max(0, generator.Generateconfig.backwardPoints));
    return (projection > backward) ? projection - backward : 0;
}

}  // namespace

int main()
{
    std::vector<rsim_driver::WorldPoint> path = BuildPath();

    rsim_driver::ReferenceLineGenerator generator;
    auto referenceLine = generator.Generate(path, path[80].x, path[80].y);
    if (!Require(referenceLine != nullptr, "reference line should be generated"))
        return 1;
    if (!Require(referenceLine->points.size() == 181,
                 "reference line should contain 181 points"))
        return 1;
    if (!Require(generator.lastProjectionIndex() == 80,
                 "first projection should match nearest path point"))
        return 1;

    const std::size_t start = ExpectedWindowStart(generator);
    for (std::size_t i = 0; i < referenceLine->points.size(); ++i)
    {
        const auto& raw = path[start + i];
        const auto& smooth = referenceLine->points[i];
        if (!Require(std::fabs(smooth.x - raw.x) <= 0.101,
                     "smoothed x should stay within bound"))
            return 1;
        if (!Require(std::fabs(smooth.y - raw.y) <= 0.101,
                     "smoothed y should stay within bound"))
            return 1;
    }

    const std::size_t firstProjection = generator.lastProjectionIndex();
    referenceLine = generator.Generate(path, path[95].x, path[95].y);
    if (!Require(referenceLine != nullptr, "second reference line should be generated"))
        return 1;
    if (!Require(generator.lastProjectionIndex() >= firstProjection,
                 "projection index should advance monotonically"))
        return 1;
    if (!Require(generator.lastProjectionIndex() == 95,
                 "second projection should match nearest path point"))
        return 1;

    const std::size_t secondStart = ExpectedWindowStart(generator);
    if (!Require(generator.lastProjectionIndex() >= secondStart,
                 "projection index should be inside generated window"))
        return 1;
    const std::size_t projectionOffset =
        generator.lastProjectionIndex() - secondStart;
    if (!Require(projectionOffset < referenceLine->points.size(),
                 "projection offset should be valid in reference line"))
        return 1;

    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    const double projectionS = referenceLine->points[projectionOffset].s;
    if (!Require(referenceLine->Eval(projectionS, 0.0, &x, &y, &heading),
                 "reference line Eval should succeed"))
        return 1;

    std::fprintf(stderr, "PASS reference_line smoke\n");
    return 0;
}
