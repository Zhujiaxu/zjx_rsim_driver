#include "DynamicSpeedPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;

        struct DynamicSpeedNode
        {
            DynamicSpeedPoint point;
            double cost = std::numeric_limits<double>::infinity();
            int previous_index = -1;
            std::size_t s_index = 0;
        };

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        double NonNegative(double value)
        {
            return std::max(0.0, value);
        }

        bool ValidConfig(const DynamicSpeedPlanConfig &config)
        {
            return IsFinite(config.time_step) &&
                   config.time_step > kEpsilon &&
                   config.time_step_count > 0 &&
                   IsFinite(config.reference_speed) &&
                   config.reference_speed >= 0.0 &&
                   IsFinite(config.weight_reference_speed) &&
                   IsFinite(config.weight_acceleration) &&
                   IsFinite(config.weight_jerk) &&
                   IsFinite(config.weight_collision);
        }

        bool ValidStart(const DynamicSpeedPlanStartPoint &start)
        {
            return IsFinite(start.s) &&
                   IsFinite(start.v) &&
                   start.v >= 0.0 &&
                   IsFinite(start.a);
        }

        std::vector<double> BuildTimeValues(const DynamicSpeedPlanConfig &config)
        {
            std::vector<double> values;
            if (!ValidConfig(config))
                return values;

            values.reserve(static_cast<std::size_t>(config.time_step_count));
            for (int i = 0; i <= config.time_step_count; ++i)
                values.push_back(static_cast<double>(i) * config.time_step);
            return values;
        }

        std::vector<double> BuildSValues(const localreferencelinepath &reference_line)
        {
            std::vector<double> values;
            if (reference_line.empty())
                return values;

            values.reserve(reference_line.size());
            for (const localreferencelinepoint &point : reference_line)
            {
                if (!IsFinite(point.s))
                    return {};
                if (!values.empty() && point.s <= values.back() + kEpsilon)
                    return {};
                values.push_back(point.s);
            }
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

        double CollisionCostAt(const DynamicSpeedPoint &point,
                               const std::vector<CutInAndOutInfo> &cut_in_and_out_infos,
                               const DynamicCollisionCostConfig &config)
        {
            if (cut_in_and_out_infos.empty())
                return 0.0;
            return DynamicObstacleCollisionCost({point.s, point.t}, cut_in_and_out_infos, config);
        }

        double SpeedCost(double speed, const DynamicSpeedPlanConfig &config)
        {
            const double delta_speed = speed - config.reference_speed;
            return NonNegative(config.weight_reference_speed) *
                   delta_speed * delta_speed;
        }

        double AccelerationCost(double acceleration,
                                const DynamicSpeedPlanConfig &config)
        {
            return NonNegative(config.weight_acceleration) *
                   acceleration * acceleration;
        }

        double JerkCost(double jerk, const DynamicSpeedPlanConfig &config)
        {
            return NonNegative(config.weight_jerk) * jerk * jerk;
        }

        double TransitionCost(double speed,
                              double acceleration,
                              double jerk,
                              double collision_cost,
                              const DynamicSpeedPlanConfig &config)
        {
            return SpeedCost(speed, config) +
                   AccelerationCost(acceleration, config) +
                   JerkCost(jerk, config) +
                   NonNegative(config.weight_collision) * collision_cost;
        }

    } // namespace

    DynamicSpeedPlanner::DynamicSpeedPlanner(
        const DynamicSpeedPlanConfig &config)
        : config_(config)
    {
    }

    const DynamicSpeedPlanConfig &DynamicSpeedPlanner::config() const
    {
        return config_;
    }

    void DynamicSpeedPlanner::SetConfig(const DynamicSpeedPlanConfig &config)
    {
        config_ = config;
    }

    bool DynamicSpeedPlanner::Plan(
        const DynamicSpeedPlanStartPoint &start,
        const localreferencelinepath &reference_line,
        const DynamicFrenetObstaclePerceptionResult &dynamic_obstacles,
        DynamicSpeedPlanResult *result) const
    {
        if (result == nullptr)
            return false;

        DynamicSpeedPlanResult output;
        const DynamicSpeedPlanConfig &config = config_;
        if (!ValidConfig(config) || !ValidStart(start))
        {
            *result = output;
            return false;
        }

        const std::vector<double> t_values = BuildTimeValues(config);
        const std::vector<double> s_values = BuildSValues(reference_line);
        if (t_values.size() < 2 ||
            s_values.size() < 2 ||
            std::fabs(s_values.front() - start.s) > kEpsilon)
        {
            *result = output;
            return false;
        }

        std::vector<CutInAndOutInfo> cut_in_and_out_infos;
        if (!compute_cut_in_and_out_.Compute(reference_line,
                                             dynamic_obstacles,
                                             &cut_in_and_out_infos))
        {
            *result = output;
            return false;
        }

        std::vector<std::vector<DynamicSpeedNode>> layers(t_values.size());
        layers.front().push_back({{
                                      0.0,
                                      start.s,
                                      start.v,
                                      start.a,
                                      0.0,
                                  },
                                  0.0,
                                  -1,
                                  0});

        for (std::size_t layer_index = 1; layer_index < t_values.size();
             ++layer_index)
        {
            std::vector<DynamicSpeedNode> &current_layer = layers[layer_index];
            current_layer.reserve(s_values.size());
            for (std::size_t s_index =s_values.size(); s_index >=0 ; --s_index)
            {
                current_layer.push_back({{
                                             t_values[layer_index],
                                             s_values[s_index],
                                             0.0,
                                             0.0,
                                             0.0,
                                         },
                                         std::numeric_limits<double>::infinity(),
                                         -1,
                                         s_index});
            }
        }

        for (std::size_t layer_index = 1; layer_index < layers.size();
             ++layer_index)
        {
            const std::vector<DynamicSpeedNode> &previous_layer =
                layers[layer_index - 1];
            std::vector<DynamicSpeedNode> &current_layer = layers[layer_index];
            const bool first_speed_column = layer_index == 1;

            for (std::size_t current_index = 0;
                 current_index < current_layer.size();
                 ++current_index)
            {
                DynamicSpeedNode &current = current_layer[current_index];
                const double collision_cost =
                    CollisionCostAt(current.point,
                                    cut_in_and_out_infos,
                                    config.collision);
                if (IsFatalCollisionCost(collision_cost, config.collision))
                    continue;

                double best_transition_cost =
                    std::numeric_limits<double>::infinity();
                int best_previous_index = -1;
                DynamicSpeedPoint best_point = current.point;

                for (std::size_t previous_index = 0;
                     previous_index < previous_layer.size();
                     ++previous_index)
                {
                    const DynamicSpeedNode &previous =
                        previous_layer[previous_index];
                    if (!std::isfinite(previous.cost))
                        continue;

                    const double delta_t = current.point.t - previous.point.t;
                    if (delta_t <= kEpsilon)
                        continue;

                    const double delta_s = current.point.s - previous.point.s;
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
                                       config);
                    if (transition_cost < best_transition_cost)
                    {
                        best_transition_cost = transition_cost;
                        best_previous_index = static_cast<int>(previous_index);
                        best_point.v = speed;
                        best_point.a = acceleration;
                        best_point.jerk = jerk;
                    }

                    if (first_speed_column)
                        break;
                }

                if (best_previous_index >= 0 &&
                    std::isfinite(best_transition_cost))
                {
                    const DynamicSpeedNode &previous =
                        previous_layer[static_cast<std::size_t>(best_previous_index)];
                    current.point = best_point;
                    current.previous_index = best_previous_index;
                    current.cost = previous.cost + best_transition_cost;
                }
            }
        }

        const std::size_t top_s_index = s_values.size();
        int best_layer_index = -1;
        int best_node_index = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (std::size_t layer_index = 1; layer_index < layers.size();
             ++layer_index)
        {
            const bool is_right_edge = layer_index + 1 == layers.size();
            for (std::size_t node_index = 0; node_index < layers[layer_index].size();
                 ++node_index)
            {
                const DynamicSpeedNode &node = layers[layer_index][node_index];
                const bool is_top_edge = node.s_index == top_s_index;
                if (!is_right_edge && !is_top_edge)
                    continue;

                if (node.cost < best_cost)
                {
                    best_cost = node.cost;
                    best_layer_index = static_cast<int>(layer_index);
                    best_node_index = static_cast<int>(node_index);
                }
            }
        }

        if (best_layer_index < 0 ||
            best_node_index < 0 ||
            !std::isfinite(best_cost))
        {
            *result = output;
            return false;
        }

        std::vector<DynamicSpeedPoint> reversed_points;
        int node_index = best_node_index;
        for (std::size_t layer_index = static_cast<std::size_t>(best_layer_index);
             layer_index > 0;
             --layer_index)
        {
            const DynamicSpeedNode &node =
                layers[layer_index][static_cast<std::size_t>(node_index)];
            reversed_points.push_back(node.point);
            node_index = node.previous_index;
            if (node_index < 0)
            {
                *result = output;
                return false;
            }
        }
        reversed_points.push_back(layers.front().front().point);
        std::reverse(reversed_points.begin(), reversed_points.end());

        output.dpsuccess = true;
        output.total_cost = best_cost;
        output.speed_points = std::move(reversed_points);
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
