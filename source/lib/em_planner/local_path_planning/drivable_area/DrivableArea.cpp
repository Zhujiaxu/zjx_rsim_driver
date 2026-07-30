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
                   IsFinite(config.approach_longitudinal_buffer) &&
                   IsFinite(config.departure_longitudinal_buffer) &&
                   IsFinite(config.obstacle_transition_length) &&
                   IsFinite(config.collision_clearance) &&
                   IsFinite(config.ego_width) &&
                   config.left_road_boundary_l >= config.right_road_boundary_l &&
                   config.obstacle_lateral_buffer >= 0.0 &&
                   config.approach_longitudinal_buffer >= 0.0 &&
                   config.departure_longitudinal_buffer >= 0.0 &&
                   config.obstacle_transition_length >= 0.0 &&
                   config.collision_clearance >= 0.0 &&
                   config.ego_width > 0.0;
        }

        double ObstacleBaseHalfLength(
            double s,
            const StaticObsFrenetState &obstacle,
            const DrivableAreaConfig &config)
        {
            return 0.5 * obstacle.length +
                   (s <= obstacle.s
                        ? config.approach_longitudinal_buffer
                        : config.departure_longitudinal_buffer);
        }

        double ObstacleConstraintRatio(
            double s,
            const StaticObsFrenetState &obstacle,
            const DrivableAreaConfig &config)
        {
            const double committedHalfLength =
                ObstacleBaseHalfLength(s, obstacle, config) +
                config.collision_clearance;
            const double distance = std::fabs(s - obstacle.s);
            if (distance <= committedHalfLength)
                return 1.0;
            if (config.obstacle_transition_length <= 0.0)
                return 0.0;
            return std::clamp(
                1.0 -
                    (distance - committedHalfLength) /
                        config.obstacle_transition_length,
                0.0, 1.0);
        }

        double ObstacleHardLateralClearance(
            double s,
            const StaticObsFrenetState &obstacle,
            const DrivableAreaConfig &config)
        {
            const double longitudinalGap = std::max(
                0.0,
                std::fabs(s - obstacle.s) -
                    ObstacleBaseHalfLength(s, obstacle, config));
            if (longitudinalGap >= config.collision_clearance ||
                config.collision_clearance <= 0.0)
            {
                return 0.0;
            }
            return std::sqrt(
                config.collision_clearance * config.collision_clearance -
                longitudinalGap * longitudinalGap);
        }

        double ObstacleHalfExtent(const StaticObsFrenetState &obstacle,
                                  const DrivableAreaConfig &config)
        {
            return 0.5 * obstacle.width +
                   config.obstacle_lateral_buffer;
        }

        bool ValidObstacle(const StaticObsFrenetState &obstacle)
        {
            return IsFinite(obstacle.s) &&
                   IsFinite(obstacle.l) &&
                   IsFinite(obstacle.length) &&
                   obstacle.length > 0.0 &&
                   IsFinite(obstacle.width) &&
                   obstacle.width > 0.0;
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
        const std::vector<StaticObsFrenetState> &staticObstacles,
        DrivableAreaResult *result) const
    {
        if (result == nullptr)
            return false;

        DrivableAreaResult output;
        if (!ValidRoadBoundary(config_) || coarsePath.empty())
        {
            output.Flag = DrivableAreaFallback::Other;
            *result = std::move(output);
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
                output.Flag = DrivableAreaFallback::Other;
                output.left_boundary.clear();
                output.right_boundary.clear();
                *result = std::move(output);
                return false;
            }
            output.left_boundary.push_back({point.s, config_.left_road_boundary_l});
            output.right_boundary.push_back({point.s, config_.right_road_boundary_l});
            previousS = point.s;
            hasPreviousS = true;
        }

        for (const StaticObsFrenetState &obstacle : staticObstacles)
        {
            if (!ValidObstacle(obstacle))
            {
                output.Flag = DrivableAreaFallback::Other;
                output.left_boundary.clear();
                output.right_boundary.clear();
                *result = std::move(output);
                return false;
            }

            const double halfLength =
                0.5 * obstacle.length +
                std::max(config_.approach_longitudinal_buffer,
                         config_.departure_longitudinal_buffer) +
                config_.collision_clearance +
                config_.obstacle_transition_length;
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
                    const double ratio = ObstacleConstraintRatio(
                        coarsePath[i].s, obstacle, config_);
                    const double obstacleBoundary =
                        obstacle.l + halfExtent +
                        ObstacleHardLateralClearance(
                            coarsePath[i].s, obstacle, config_);
                    const double transitionedBoundary =
                        config_.right_road_boundary_l +
                        ratio * (obstacleBoundary -
                                 config_.right_road_boundary_l);
                    output.right_boundary[i].l =
                        std::max(output.right_boundary[i].l,
                                 transitionedBoundary);
                }
            }

            else
            {
                for (std::size_t i = firstCoveredIndex; i < afterCoveredIndex; ++i)
                {
                    const double ratio = ObstacleConstraintRatio(
                        coarsePath[i].s, obstacle, config_);
                    const double obstacleBoundary =
                        obstacle.l - halfExtent -
                        ObstacleHardLateralClearance(
                            coarsePath[i].s, obstacle, config_);
                    const double transitionedBoundary =
                        config_.left_road_boundary_l +
                        ratio * (obstacleBoundary -
                                 config_.left_road_boundary_l);
                    output.left_boundary[i].l =
                        std::min(output.left_boundary[i].l,
                                 transitionedBoundary);
                }
            }
        }

        for (std::size_t i = 0; i < coarsePath.size(); ++i)
        {
            if (output.right_boundary[i].l+config_.ego_width > output.left_boundary[i].l)
            {
                output.Flag = DrivableAreaFallback::Stop;
                output.left_boundary.clear();
                output.right_boundary.clear();
                *result = std::move(output);
                return false;
            }
        }

        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
