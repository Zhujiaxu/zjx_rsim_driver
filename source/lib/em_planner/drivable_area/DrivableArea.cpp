#include "drivable_area/DrivableArea.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

namespace
{

bool IsFinite(double value)
{
    return std::isfinite(value);
}

bool ValidRoadBoundary(const DrivableAreaConfig& config)
{
    return IsFinite(config.left_road_boundary_l) &&
           IsFinite(config.right_road_boundary_l) &&
           IsFinite(config.obstacle_lateral_buffer) &&
           config.left_road_boundary_l >= config.right_road_boundary_l &&
           config.obstacle_lateral_buffer >= 0.0;
}

double ObstacleHalfLength(const StaticFrenetObstacle& obstacle)
{
    return 0.5 * std::max(0.0, obstacle.length);
}

double ObstacleHalfExtent(const StaticFrenetObstacle& obstacle,
                          const DrivableAreaConfig& config)
{
    return 0.5 * std::max(std::max(0.0, obstacle.length),
                          std::max(0.0, obstacle.width)) +
           config.obstacle_lateral_buffer;
}

bool ObstacleCoversS(const StaticFrenetObstacle& obstacle, double s)
{
    const double halfLength = ObstacleHalfLength(obstacle);
    return s >= obstacle.s - halfLength && s <= obstacle.s + halfLength;
}

}  // namespace

DrivableAreaBuilder::DrivableAreaBuilder(const DrivableAreaConfig& config)
    : config_(config)
{
}

const DrivableAreaConfig& DrivableAreaBuilder::config() const
{
    return config_;
}

void DrivableAreaBuilder::SetConfig(const DrivableAreaConfig& config)
{
    config_ = config;
}

bool DrivableAreaBuilder::Build(
    const std::vector<DpPathPoint>& coarsePath,
    const std::vector<StaticFrenetObstacle>& staticObstacles,
    DrivableArea* result) const
{
    if (result == nullptr)
        return false;

    DrivableArea output;
    if (!ValidRoadBoundary(config_) || coarsePath.empty())
    {
        *result = output;
        return false;
    }

    output.left_boundary.reserve(coarsePath.size());
    output.right_boundary.reserve(coarsePath.size());
    for (const DpPathPoint& point : coarsePath)
    {
        if (!IsFinite(point.s) || !IsFinite(point.l))
        {
            *result = DrivableArea{};
            return false;
        }
        output.left_boundary.push_back({point.s, config_.left_road_boundary_l});
        output.right_boundary.push_back({point.s, config_.right_road_boundary_l});
    }

    for (std::size_t i = 0; i < coarsePath.size(); ++i)
    {
        const DpPathPoint& coarsePoint = coarsePath[i];
        for (const StaticFrenetObstacle& obstacle : staticObstacles)
        {
            if (!IsFinite(obstacle.s) ||
                !IsFinite(obstacle.l) ||
                !IsFinite(obstacle.length) ||
                !IsFinite(obstacle.width) ||
                !ObstacleCoversS(obstacle, coarsePoint.s))
            {
                continue;
            }

            const double halfExtent = ObstacleHalfExtent(obstacle, config_);
            if (coarsePoint.l >= obstacle.l)
            {
                output.right_boundary[i].l =
                    std::max(output.right_boundary[i].l,
                             obstacle.l + halfExtent);
            }
            else
            {
                output.left_boundary[i].l =
                    std::min(output.left_boundary[i].l,
                             obstacle.l - halfExtent);
            }
        }

        if (output.right_boundary[i].l > output.left_boundary[i].l)
        {
            *result = DrivableArea{};
            return false;
        }
    }

    *result = std::move(output);
    return true;
}

}  // namespace rsim_driver
