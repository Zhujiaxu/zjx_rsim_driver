#pragma once

#include "CollisionCost.hpp"
#include "DynamicSpeedPlanner.hpp"
#include "computecutinandout.hpp"

#include <vector>

namespace rsim_driver
{

struct StDrivableAreaConfig
{
    double longitudinal_safety_buffer = 6.0;
};

struct StDrivableArea
{
    std::vector<StPoint> lower_boundary;
    std::vector<StPoint> upper_boundary;
};

class StDrivableAreaBuilder
{
public:
    explicit StDrivableAreaBuilder(const StDrivableAreaConfig &config = {});

    const StDrivableAreaConfig &config() const;
    void SetConfig(const StDrivableAreaConfig &config);

    bool Build(const std::vector<CutInAndOutInfo> &st_boundary_infos,
               const DynamicPlanSpeedPoint& start,
               const DynamicPlanSpeedConfig &plan_config,
               double reference_line_total_length,
               StDrivableArea *result) const;

private:
    StDrivableAreaConfig config_;
};

}  // namespace rsim_driver
