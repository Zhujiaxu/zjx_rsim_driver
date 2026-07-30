#pragma once

#include "CollisionCost.hpp"
#include "DpPlanner.hpp"
#include <vector>

namespace rsim_driver
{

    struct DrivableAreaConfig
    {
        double left_road_boundary_l =4.5;
        double right_road_boundary_l = -4.5;
        double obstacle_lateral_buffer = 0.05;
        // Extra longitudinal guards around the physical obstacle envelope.
        // Departure is held longer for receding-horizon stability.
        double approach_longitudinal_buffer = 0.0;
        double departure_longitudinal_buffer = 1.9;
        double obstacle_transition_length = 5.0;
        double collision_clearance = 0.6;
        double ego_width = 2.0;
    };
    enum class DrivableAreaFallback
    {
        Success,
        Stop,
        Other,
    };

    struct DrivableAreaResult
    {
        DrivableAreaFallback Flag = DrivableAreaFallback::Success;
        std::vector<SlPoint> left_boundary;
        std::vector<SlPoint> right_boundary;
    };

    class DrivableAreaBuilder
    {
    public:
        explicit DrivableAreaBuilder(const DrivableAreaConfig &config = {});

        const DrivableAreaConfig &config() const;
        void SetConfig(const DrivableAreaConfig &config);

        bool Build(const std::vector<DpPathPoint> &coarsePath,
                   const std::vector<StaticObsFrenetState> &staticObstacles,
                   DrivableAreaResult *result) const;

    private:
        DrivableAreaConfig config_;
    };

} // namespace rsim_driver
