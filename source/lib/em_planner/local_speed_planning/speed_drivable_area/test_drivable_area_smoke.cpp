#include "StDrivableArea.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

rsim_driver::DynamicPlanSpeedResult MakeCoarse(double middle_s = 2.0,
                                               double terminal_s = 3.0)
{
    rsim_driver::DynamicPlanSpeedResult result;
    result.Flag = rsim_driver::DynamicPlanSpeedFallback::Success;
    result.total_s = 20.0;
    result.stpoints = {
        {0.0, 0.0, 0.0, 0.0},
        {1.0, middle_s, 0.0, 0.0},
        {2.0, terminal_s, 0.0, 0.0},
    };
    return result;
}

rsim_driver::CutInAndOutInfo StationaryBoundary(double minimum,
                                                 double maximum)
{
    rsim_driver::CutInAndOutInfo info;
    info.id = 10;
    info.tin = 0.0;
    info.tout = 2.0;
    info.sin = 0.5 * (minimum + maximum);
    info.sinmin = minimum;
    info.sinmax = maximum;
    info.sout = info.sin;
    info.soutmin = minimum;
    info.soutmax = maximum;
    return info;
}

}  // namespace

int main()
{
    rsim_driver::StDrivableAreaBuilder builder;
    rsim_driver::StDrivableAreaResult area;
    const std::vector<double> grid{0.0, 1.0, 2.0};
    const auto coarse = MakeCoarse();
    if (!Require(builder.Build({}, &coarse, &area) &&
                     area.lower_boundary.size() == 3 &&
                     area.upper_boundary.size() == 3 &&
                     area.lower_is_obstacle.size() == 3 &&
                     area.upper_is_obstacle.size() == 3,
                 "empty obstacle set should produce a full corridor"))
        return 1;
    for (std::size_t i = 0; i < grid.size(); ++i)
    {
        if (!Require(std::fabs(area.lower_boundary[i].t - grid[i]) < 1e-9 &&
                         std::fabs(area.upper_boundary[i].t - grid[i]) < 1e-9 &&
                         area.lower_boundary[i].s == -2.5 &&
                         area.upper_boundary[i].s == 20.0 &&
                         area.lower_is_obstacle[i] == 0U &&
                         area.upper_is_obstacle[i] == 0U,
                     "corridor should preserve the explicit time grid"))
            return 1;
    }

    const auto ahead = StationaryBoundary(4.0, 6.0);
    if (!Require(builder.Build({ahead}, &coarse, &area) &&
                     area.upper_boundary[1].s == 4.0 &&
                     area.upper_is_obstacle[1] == 1U,
                 "coarse path behind obstacle should select the lower ST side"))
        return 1;

    const auto crossing = MakeCoarse(5.0, 5.0);
    if (!Require(!builder.Build({ahead}, &crossing, &area) &&
                     area.lower_boundary.empty(),
                 "coarse path crossing an obstacle should fail"))
        return 1;

    auto invalid = ahead;
    invalid.sinmin = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!builder.Build({invalid}, &coarse, &area) &&
                     area.lower_boundary.empty() &&
                     area.upper_boundary.empty(),
                 "invalid ST boundary should fail closed"))
        return 1;
    if (!Require(!builder.Build({}, &coarse, nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS st_drivable_area smoke\n");
    return 0;
}
