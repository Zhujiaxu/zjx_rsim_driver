#pragma once

#include "CollisionCost.hpp"
#include "DynamicSpeedPlanner.hpp"
#include "computecutinandout.hpp"

#include <vector>

namespace rsim_driver
{
    enum class StDrivableAreaFallback
    {
        Success,
        Other,
        Stop,
    };

    struct StDrivableAreaConfig
    {
        double longitudinal_safety_buffer = 5.0; // meters
    };

    struct StDrivableAreaResult
    {
        StDrivableAreaFallback Flag = StDrivableAreaFallback::Success;
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
                   const DynamicPlanSpeedResult *dpplan_result,
                   StDrivableAreaResult *result) const;

    private:
        StDrivableAreaConfig config_;
    };

} // namespace rsim_driver
