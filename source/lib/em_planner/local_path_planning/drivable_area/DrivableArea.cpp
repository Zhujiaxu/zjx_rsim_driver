#include "DrivableArea.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace rsim_driver
{

    namespace
    {

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        bool ValidRoadBoundary(const DrivableAreaConfig &config)
        {
            return IsFinite(config.left_road_boundary_l) &&
                   IsFinite(config.right_road_boundary_l) &&
                   IsFinite(config.obstacle_lateral_buffer) &&
                   config.left_road_boundary_l >= config.right_road_boundary_l &&
                   config.obstacle_lateral_buffer >= 0.0;
        }

        double ObstacleHalfLength(const StaticFrenetObstacle &obstacle)
        {
            return 0.5 * std::max(0.0, obstacle.length);
        }

        double ObstacleHalfExtent(const StaticFrenetObstacle &obstacle,
                                  const DrivableAreaConfig &config)
        {
            return 0.5 * std::max(0.0, obstacle.width) +
                   config.obstacle_lateral_buffer;
        }

        bool ValidObstacle(const StaticFrenetObstacle &obstacle)
        {
            return IsFinite(obstacle.s) &&
                   IsFinite(obstacle.l) &&
                   IsFinite(obstacle.length) &&
                   IsFinite(obstacle.width);
        }

        std::size_t IndexAtOrAfterS(const std::vector<DpPathPoint> &coarsePath,
                                    double s)
        {
            const auto it = std::lower_bound(
                coarsePath.begin(),
                coarsePath.end(),
                s,
                [](const DpPathPoint &point, double value)
                {
                    return point.s < value;
                });
            return static_cast<std::size_t>(std::distance(coarsePath.begin(), it));
        }

        std::size_t IndexAfterS(const std::vector<DpPathPoint> &coarsePath, double s)
        {
            const auto it = std::upper_bound(
                coarsePath.begin(),
                coarsePath.end(),
                s,
                [](double value, const DpPathPoint &point)
                {
                    return value < point.s;
                });
            return static_cast<std::size_t>(std::distance(coarsePath.begin(), it));
        }

        double AverageCoarsePathLAtObstacleS(
            const std::vector<DpPathPoint> &coarsePath,
            double obstacleS)
        {
            if (coarsePath.size() == 1)
                return coarsePath.front().l;

            const std::size_t nextIndex = IndexAtOrAfterS(coarsePath, obstacleS);
            std::size_t firstIndex = 0;
            if (nextIndex == 0)
                firstIndex = 0;
            else if (nextIndex >= coarsePath.size())
                firstIndex = coarsePath.size() - 2;
            else
                firstIndex = nextIndex - 1;

            return 0.5 * (coarsePath[firstIndex].l + coarsePath[firstIndex + 1].l);
        }

    } // namespace

    DrivableAreaBuilder::DrivableAreaBuilder(const DrivableAreaConfig &config)
        : config_(config)
    {
    }

    const DrivableAreaConfig &DrivableAreaBuilder::config() const
    {
        return config_;
    }

    void DrivableAreaBuilder::SetConfig(const DrivableAreaConfig &config)
    {
        config_ = config;
    }

    bool DrivableAreaBuilder::Build(
        const std::vector<DpPathPoint> &coarsePath,
        const std::vector<StaticFrenetObstacle> &staticObstacles,
        DrivableArea *result) const
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
        double previousS = 0.0;
        bool hasPreviousS = false;
        for (const DpPathPoint &point : coarsePath)
        {
            if (!IsFinite(point.s) ||
                !IsFinite(point.l) ||
                (hasPreviousS && point.s < previousS))
            {
                *result = DrivableArea{};
                return false;
            }
            output.left_boundary.push_back({point.s, config_.left_road_boundary_l});
            output.right_boundary.push_back({point.s, config_.right_road_boundary_l});
            previousS = point.s;
            hasPreviousS = true;
        }

        for (const StaticFrenetObstacle &obstacle : staticObstacles)
        {
            if (!ValidObstacle(obstacle))
                continue;

            const double halfLength = ObstacleHalfLength(obstacle);
            const std::size_t firstCoveredIndex =
                IndexAtOrAfterS(coarsePath, obstacle.s - halfLength);
            const std::size_t afterCoveredIndex =
                IndexAfterS(coarsePath, obstacle.s + halfLength);
            if (firstCoveredIndex >= afterCoveredIndex)
                continue;

            const double averageCoarseL =
                AverageCoarsePathLAtObstacleS(coarsePath, obstacle.s);
            const double halfExtent = ObstacleHalfExtent(obstacle, config_);
            const bool coarsePathLeftOfObstacle = averageCoarseL >= obstacle.l;
            if (coarsePathLeftOfObstacle)
            {
                for (std::size_t i = firstCoveredIndex; i < afterCoveredIndex; ++i)
                {
                    output.right_boundary[i].l =
                        std::max(output.right_boundary[i].l, obstacle.l + halfExtent);
                }
            }

            else
            {
                for (std::size_t i = firstCoveredIndex; i < afterCoveredIndex; ++i)
                {
                    output.left_boundary[i].l =
                        std::min(output.left_boundary[i].l,
                                 obstacle.l - halfExtent);
                }
            }
        }

        for (std::size_t i = 0; i < coarsePath.size(); ++i)
        {
            if (output.right_boundary[i].l+config_.ego_width > output.left_boundary[i].l)
            {
                *result = DrivableArea{};
                return false;
            }
        }

        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
