#include "DynamicSpeedPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;

        struct DPNode
        {
            DynamicPlanSpeedPoint point;
            double cost = std::numeric_limits<double>::infinity();
            double jerk = 0.0;
            int previous_index = -1;
        };

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        double NonNegative(double value)
        {
            return std::max(0.0, value);
        }

        bool ValidConfig(const DynamicPlanSpeedConfig &config)
        {
            return IsFinite(config.time_step) &&
                   config.time_step > kEpsilon &&
                   config.time_step_count > 0 &&
                   IsFinite(config.s_step) &&
                   config.s_step > kEpsilon &&
                   config.s_step_count > 0 &&
                   IsFinite(config.planning_period) &&
                   config.planning_period >= 0.0 &&
                   IsFinite(config.reference_speed) &&
                   config.reference_speed >= 0.0 &&
                   IsFinite(config.weight_reference_speed) &&
                   IsFinite(config.weight_acceleration) &&
                   IsFinite(config.weight_jerk) &&
                   IsFinite(config.weight_collision);
        }

        bool ValidStart(const DynamicPlanSpeedPoint &start)
        {
            return IsFinite(start.s) &&
                   IsFinite(start.v) &&
                   start.v >= 0.0 &&
                   IsFinite(start.a);
        }

        std::vector<double> BuildTimeValues(const DynamicPlanSpeedConfig &config)
        {
            std::vector<double> values;
            if (!ValidConfig(config))
                return values;
            for (int i = 0; i <= config.time_step_count; ++i)
                values.push_back(static_cast<double>(i) * config.time_step);
            return values;
        }

        std::vector<double> BuildSValues(const DynamicPlanSpeedConfig &config,
                                         double maxS)
        {
            std::vector<double> values;
            if (!ValidConfig(config) || std::isnan(maxS))
                return values;

            const double nominalEnd =
                static_cast<double>(config.s_step_count) * config.s_step;
            const double endS = std::isfinite(maxS)
                                    ? std::min(nominalEnd, maxS)
                                    : nominalEnd;
            if (!std::isfinite(endS) || endS <= kEpsilon)
                return values;

            const int intervalCount = std::min(
                config.s_step_count,
                std::max(1, static_cast<int>(std::ceil(endS / config.s_step))));
            const double step = endS / static_cast<double>(intervalCount);
            values.reserve(static_cast<std::size_t>(intervalCount) + 1U);
            for (int i = 0; i <= intervalCount; ++i)
                values.push_back(static_cast<double>(i) * step);
            values.back() = endS;
            return values;
        }

        bool IsFatalCollisionCost(double cost, const DynamicCollisionCostConfig &config)
        {
            if (!std::isfinite(cost))
                return true;
            if (std::isfinite(config.infinity_cost) && cost == config.infinity_cost)
                return true;
            return false;
        }

        double CollisionCostAt(const DynamicPlanSpeedPoint &point,
                               const std::vector<CutInAndOutInfo> &cut_in_and_out_infos,
                               const DynamicCollisionCostConfig &config)
        {
            if (cut_in_and_out_infos.empty())
                return 0.0;
            return DynamicObstacleCollisionCost({point.s, point.t},
                                                cut_in_and_out_infos,
                                                config);
        }

        double SpeedCost(double speed, const DynamicPlanSpeedConfig &config)
        {
            const double delta_speed = speed - config.reference_speed;
            return NonNegative(config.weight_reference_speed) *
                   delta_speed * delta_speed;
        }

        double AccelerationCost(double acceleration,
                                const DynamicPlanSpeedConfig &config)
        {
            return NonNegative(config.weight_acceleration) *
                   acceleration * acceleration;
        }

        double JerkCost(double jerk, const DynamicPlanSpeedConfig &config)
        {
            return NonNegative(config.weight_jerk) * jerk * jerk;
        }

        double TransitionCost(double speed,
                              double acceleration,
                              double jerk,
                              double collision_cost,
                              const DynamicPlanSpeedConfig &config)
        {
            return SpeedCost(speed, config) +
                   AccelerationCost(acceleration, config) +
                   JerkCost(jerk, config) +
                   NonNegative(config.weight_collision) * collision_cost;
        }

    } // namespace

    DynamicPlanSpeedPlanner::DynamicPlanSpeedPlanner(
        const DynamicPlanSpeedConfig &config)
        : config_(config)
    {
    }

    const DynamicPlanSpeedConfig &DynamicPlanSpeedPlanner::config() const
    {
        return config_;
    }

    void DynamicPlanSpeedPlanner::SetConfig(const DynamicPlanSpeedConfig &config)
    {
        config_ = config;
    }

    bool DynamicPlanSpeedPlanner::Plan(
        const DynamicPlanSpeedPoint &start,
        const std::vector<CutInAndOutInfo> &STBoundaryInfos,
        DynamicPlanSpeedResult *result,
        double max_s,
        bool stop_at_end) const
    {
        if (result == nullptr)
            return false;

        DynamicPlanSpeedResult output;
        const DynamicPlanSpeedConfig &config = config_;
        if (!ValidConfig(config) || !ValidStart(start))
        {
            output.Flag = DynamicPlanSpeedFallback::Other;
            *result = std::move(output);
            return false;
        }

        const std::vector<double> t_values = BuildTimeValues(config);
        const std::vector<double> s_values = BuildSValues(config, max_s);
        if (t_values.size() < 2 ||
            s_values.size() < 2 ||
            std::fabs(s_values.front() - start.s) > kEpsilon)
        {
            output.Flag = DynamicPlanSpeedFallback::Other;
            *result = std::move(output);
            return false;
        }

        std::vector<std::vector<DPNode>> layers(t_values.size());
        layers.front().push_back({
            {
                0.0,
                start.s,
                start.v,
                start.a,
            },
            0.0,
            0.0,
            -1,
        });

        for (std::size_t layer_index = 1; layer_index < t_values.size();
             ++layer_index)
        {
            std::vector<DPNode> &current_layer = layers[layer_index];
            current_layer.reserve(s_values.size());
            for (std::size_t s_index = 0; s_index < s_values.size(); ++s_index)
            {
                current_layer.push_back({{
                                             t_values[layer_index],
                                             s_values[s_index],
                                             0.0,
                                             0.0,
                                         },
                                         std::numeric_limits<double>::infinity(),
                                         0.0,
                                         -1});
            }
        }

        for (std::size_t layer_index = 1; layer_index < layers.size();
             ++layer_index)
        {
            const std::vector<DPNode> &previous_layer =
                layers[layer_index - 1];
            std::vector<DPNode> &current_layer = layers[layer_index];
            // const bool first_speed_column = layer_index == 1;

            for (std::size_t current_index = 0;
                 current_index < current_layer.size();
                 ++current_index)
            {
                DPNode &curnode = current_layer[current_index];
                const double collision_cost =
                    CollisionCostAt(curnode.point,
                                    STBoundaryInfos,
                                    config.collisionconfig);
                if (IsFatalCollisionCost(collision_cost, config.collisionconfig))
                    continue;

                int best_previous_index = -1;

                for (std::size_t previous_index = 0;
                     previous_index < previous_layer.size();
                     ++previous_index)
                {
                    const DPNode &previous =
                        previous_layer[previous_index];
                    if (!std::isfinite(previous.cost))
                        continue;

                    const double delta_t = curnode.point.t - previous.point.t;
                    if (delta_t <= kEpsilon)
                        continue;

                    const double delta_s = curnode.point.s - previous.point.s;
                    if (delta_s < -kEpsilon)
                        continue;

                    const double speed = std::max(0.0, delta_s / delta_t);
                    const double acceleration =
                        (speed - previous.point.v) / delta_t;
                    const double jerk =
                        (acceleration - previous.point.a) / delta_t;

                    const double transition_cost =
                        TransitionCost(speed,
                                       acceleration,
                                       jerk,
                                       collision_cost,
                                       config) +
                        previous.cost;
                    if (transition_cost < curnode.cost)
                    {
                        curnode.cost = transition_cost;
                        curnode.previous_index = static_cast<int>(previous_index);
                        curnode.point.v = speed;
                        curnode.point.a = acceleration;
                        curnode.jerk = jerk;
                    }
                }
            }
        }
        int best_layer_index = -1;
        int best_node_index = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        if (stop_at_end)
        {
            const std::size_t layer_index = layers.size() - 1U;
            const std::size_t node_index = layers[layer_index].size() - 1U;
            const DPNode &node = layers[layer_index][node_index];
            if (std::isfinite(node.cost))
            {
                best_cost = node.cost;
                best_layer_index = static_cast<int>(layer_index);
                best_node_index = static_cast<int>(node_index);
            }
        }
        else
        {
            for (std::size_t layer_index = 1; layer_index < layers.size();
                 ++layer_index)
            {
                const bool is_right_edge = layer_index + 1 == layers.size();
                if (is_right_edge)
                {
                    for (std::size_t node_index = 0;
                         node_index < layers[layer_index].size();
                         ++node_index)
                    {
                        const DPNode &node = layers[layer_index][node_index];
                        if (node.cost < best_cost)
                        {
                            best_cost = node.cost;
                            best_layer_index = static_cast<int>(layer_index);
                            best_node_index = static_cast<int>(node_index);
                        }
                    }
                    break;
                }
                const std::size_t node_index = layers[layer_index].size() - 1U;
                const double cost = layers[layer_index][node_index].cost;
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_layer_index = static_cast<int>(layer_index);
                    best_node_index = static_cast<int>(node_index);
                }
            }
        }

        if (best_layer_index < 0 ||
            best_node_index < 0 ||
            !std::isfinite(best_cost))
        {
            output.Flag = DynamicPlanSpeedFallback::Stop;
            *result = std::move(output);
            return false;
        }

        std::vector<DynamicPlanSpeedPoint> reversed_points;
        int node_index = best_node_index;
        for (std::size_t layer_index = static_cast<std::size_t>(best_layer_index);
             layer_index > 0;
             --layer_index)
        {
            const DPNode &node =
                layers[layer_index][static_cast<std::size_t>(node_index)];
            reversed_points.push_back(node.point);
            node_index = node.previous_index;
        }
        reversed_points.push_back(layers.front().front().point);
        std::reverse(reversed_points.begin(), reversed_points.end());

        output.Flag = DynamicPlanSpeedFallback::Success;
        output.total_cost = best_cost;
        output.total_s = reversed_points.back().s;
        output.stpoints = std::move(reversed_points);
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
