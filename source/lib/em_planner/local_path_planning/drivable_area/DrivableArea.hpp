#pragma once

#include "CollisionCost.hpp"
#include "DpPlanner.hpp"
#include <vector>

namespace rsim_driver
{

    struct DrivableAreaConfig
    {
        double left_road_boundary_l =3.5;
        double right_road_boundary_l = -3.5;
        double obstacle_lateral_buffer = 0.4;
    };

    struct DrivableArea
    {
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
                   const std::vector<StaticFrenetObstacle> &staticObstacles,
                   DrivableArea *result) const;

    private:
        DrivableAreaConfig config_;
    };

} // namespace rsim_driver
