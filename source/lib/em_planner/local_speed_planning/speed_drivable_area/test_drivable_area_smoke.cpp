#include "StDrivableArea.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DynamicPlanSpeedConfig MakePlanConfig(
    double time_step = 1.0,
    int time_step_count = 4)
{
    rsim_driver::DynamicPlanSpeedConfig config;
    config.time_step = time_step;
    config.time_step_count = time_step_count;
    return config;
}

rsim_driver::CutInAndOutInfo MakeCutInAndOut(
    int32_t id,
    double tin, double tout,
    double sinmin, double sinmax,
    double soutmin, double soutmax)
{
    rsim_driver::CutInAndOutInfo info;
    info.id = id;
    info.tin = tin;
    info.tout = tout;
    info.sin = (sinmin + sinmax) * 0.5;
    info.sinmin = sinmin;
    info.sinmax = sinmax;
    info.sout = (soutmin + soutmax) * 0.5;
    info.soutmin = soutmin;
    info.soutmax = soutmax;
    return info;
}

}  // namespace

int main()
{
    rsim_driver::StDrivableAreaConfig config;
    config.longitudinal_safety_buffer = 2.0;
    config.ego_initial_speed = 0.0;
    rsim_driver::StDrivableAreaBuilder builder(config);
    rsim_driver::StDrivableArea area;

    // --- Test 1: No obstacles ---
    if (!Require(builder.Build({}, MakePlanConfig(), 20.0, &area),
                 "empty obstacles should build drivable area"))
        return 1;
    if (!Require(area.lower_boundary.size() == 5 &&
                     area.upper_boundary.size() == 5,
                 "output should have time_step_count+1 entries"))
        return 1;
    for (std::size_t i = 0; i < area.lower_boundary.size(); ++i)
    {
        if (!Require(
                Near(area.lower_boundary[i].s, 0.0) &&
                Near(area.upper_boundary[i].s, 20.0) &&
                Near(area.lower_boundary[i].t, static_cast<double>(i)) &&
                Near(area.upper_boundary[i].t, static_cast<double>(i)),
                "no obstacles should keep full bounds"))
            return 1;
    }

    // --- Test 2: Single obstacle cutting in ---
    const std::vector<rsim_driver::CutInAndOutInfo> singleObstacle = {
        MakeCutInAndOut(1, 1.0, 3.0, 8.0, 10.0, 14.0, 16.0),
    };
    if (!Require(builder.Build(singleObstacle, MakePlanConfig(), 20.0, &area),
                 "single obstacle ahead should build"))
        return 1;

    // t=0: no obstacle
    if (!Require(Near(area.upper_boundary[0].s, 20.0),
                 "t=0 should have full upper bound"))
        return 1;

    // t=1: obstacle at [8,10], upper should be 8 - 2 = 6
    if (!Require(Near(area.upper_boundary[1].s, 6.0),
                 "t=1 should shrink upper to sinmin - buffer"))
        return 1;

    // t=2: obstacle at [11,13], upper should be 11 - 2 = 9
    if (!Require(Near(area.upper_boundary[2].s, 9.0),
                 "t=2 should shrink upper to interpolated sinmin - buffer"))
        return 1;

    // t=3: obstacle at [14,16], upper should be 14 - 2 = 12
    if (!Require(Near(area.upper_boundary[3].s, 12.0),
                 "t=3 should shrink upper at tout"))
        return 1;

    // t=4: no obstacle anymore
    if (!Require(Near(area.upper_boundary[4].s, 20.0),
                 "t=4 should restore full upper bound"))
        return 1;

    for (const auto &pt : area.lower_boundary)
    {
        if (!Require(Near(pt.s, 0.0),
                     "lower bound should stay at 0 for ahead obstacles"))
            return 1;
    }

    // --- Test 3: Multiple obstacles ---
    const std::vector<rsim_driver::CutInAndOutInfo> multipleObstacles = {
        MakeCutInAndOut(1, 1.0, 3.0, 8.0, 10.0, 14.0, 16.0),
        MakeCutInAndOut(2, 1.0, 3.0, 5.0, 7.0, 5.0, 7.0),
    };
    if (!Require(builder.Build(multipleObstacles, MakePlanConfig(), 20.0, &area),
                 "multiple obstacles should build"))
        return 1;

    // The more restrictive obstacle 2 caps upper at 5 - 2 = 3 at t=1
    if (!Require(Near(area.upper_boundary[1].s, 3.0),
                 "most restrictive ahead obstacle should win"))
        return 1;

    // --- Test 4: Obstacle that completely blocks ---
    const std::vector<rsim_driver::CutInAndOutInfo> blockingObstacle = {
        MakeCutInAndOut(1, 0.0, 4.0, 0.0, 2.0, 0.0, 2.0),
    };
    if (!Require(!builder.Build(blockingObstacle, MakePlanConfig(), 20.0, &area) &&
                     area.lower_boundary.empty() &&
                     area.upper_boundary.empty(),
                 "blocking obstacle should fail and clear output"))
        return 1;

    // --- Test 5: Invalid config ---
    rsim_driver::StDrivableAreaConfig invalidConfig;
    invalidConfig.longitudinal_safety_buffer = -1.0;
    rsim_driver::StDrivableAreaBuilder invalidBuilder(invalidConfig);
    if (!Require(!invalidBuilder.Build({}, MakePlanConfig(), 20.0, &area) &&
                     area.lower_boundary.empty(),
                 "negative safety buffer should fail"))
        return 1;

    // --- Test 6: Null result ---
    if (!Require(!builder.Build({}, MakePlanConfig(), 20.0, nullptr),
                 "null result should fail"))
        return 1;

    // --- Test 7: Invalid plan config ---
    auto badPlanCfg = MakePlanConfig(0.0, 4);
    if (!Require(!builder.Build({}, badPlanCfg, 20.0, &area),
                 "zero time step should fail"))
        return 1;

    // --- Test 8: Zero reference line length ---
    if (!Require(!builder.Build({}, MakePlanConfig(), 0.0, &area),
                 "zero reference line length should fail"))
        return 1;

    // --- Test 9: Obstacle behind ego ---
    rsim_driver::StDrivableAreaConfig fastEgoConfig;
    fastEgoConfig.longitudinal_safety_buffer = 2.0;
    fastEgoConfig.ego_initial_speed = 10.0;
    rsim_driver::StDrivableAreaBuilder fastBuilder(fastEgoConfig);

    const std::vector<rsim_driver::CutInAndOutInfo> behindObstacle = {
        MakeCutInAndOut(1, 1.0, 3.0, 5.0, 7.0, 5.0, 7.0),
    };
    if (!Require(fastBuilder.Build(behindObstacle, MakePlanConfig(), 20.0, &area),
                 "obstacle behind fast ego should build"))
        return 1;

    // At t=1, ego is at ~10, obstacle at [5,7] behind ego
    // Lower bound should be 7 + 2 = 9
    if (!Require(Near(area.lower_boundary[1].s, 9.0),
                 "obstacle behind should raise lower bound"))
        return 1;

    std::fprintf(stderr, "PASS drivable_area smoke\n");
    return 0;
}
