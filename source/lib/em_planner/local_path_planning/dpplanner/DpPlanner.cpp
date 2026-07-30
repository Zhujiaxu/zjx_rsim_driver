#include "DpPlanner.hpp"

#include "QuinticPolynomial.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;
        constexpr int kSegmentCostSamples = 3;

        struct DpNode
        {
            DpPathPoint point;
            double cost = std::numeric_limits<double>::infinity();
            int previous_index = -1;
        };

        std::vector<double> BuildSValues(double startS,
                                         const DpPlannerConfig &config,
                                         double maxS)
        {
            std::vector<double> values;
            if (!std::isfinite(startS) || std::isnan(maxS) ||
                config.s_step <= kEpsilon || config.s_step_count <= 0)
                return values;

            const double nominalEnd =
                startS + static_cast<double>(config.s_step_count) * config.s_step;
            const double endS = std::isfinite(maxS)
                                    ? std::min(nominalEnd, maxS)
                                    : nominalEnd;
            const double span = endS - startS;
            if (!std::isfinite(endS) || span <= kEpsilon)
                return values;

            const int intervalCount = std::min(
                config.s_step_count,
                std::max(1, static_cast<int>(std::ceil(span / config.s_step))));
            const double step = span / static_cast<double>(intervalCount);
            values.reserve(static_cast<std::size_t>(intervalCount) + 1U);
            for (int i = 0; i <= intervalCount; ++i)
                values.push_back(startS + static_cast<double>(i) * step);
            values.back() = endS;
            return values;
        }

        std::vector<double> BuildLSamples(double centerL, const DpPlannerConfig &config)
        {
            std::vector<double> samples;
            if (!std::isfinite(centerL) ||
                config.l_step <= kEpsilon ||
                config.left_l_step_count < 0 ||
                config.right_l_step_count < 0)
            {
                return samples;
            }

            samples.reserve(static_cast<std::size_t>(
                config.left_l_step_count + config.right_l_step_count + 1));
            samples.push_back(centerL);
            for (int i = 1; i <= config.left_l_step_count; ++i)
                samples.push_back(centerL + static_cast<double>(i) * config.l_step);
            for (int i = 1; i <= config.right_l_step_count; ++i)
                samples.push_back(centerL - static_cast<double>(i) * config.l_step);

            std::sort(samples.begin(), samples.end());
            return samples;
        }

        bool IsFatalCollisionCost(double cost, const StaticCollisionCostConfig &config)
        {
            if (!std::isfinite(cost))
                return true;
            if (std::isfinite(config.infinity_cost) && cost == config.infinity_cost)
                return true;
            return false;
        }

        double ReferenceNodeCost(double l, const DpPlannerConfig &config)
        {
            return std::max(0.0, config.weight_ref_l) * l * l;
        }

        double SmoothSegmentCost(double deltaS,
                                 const QuinticBoundary &start,
                                 const QuinticBoundary &end,
                                 const DpPlannerConfig &config)
        {
            const double wLPrime = std::max(0.0, config.weight_l_prime);
            const double wLDoublePrime = std::max(0.0, config.weight_l_double_prime);

            QuinticPolynomial1d polynomial;
            if (!QuinticPolynomial1d::Create(deltaS, start, end, &polynomial))
                return std::numeric_limits<double>::infinity();

            double cost = 0.0;
            for (int i = 1; i <= kSegmentCostSamples; ++i)
            {
                const double s = deltaS * static_cast<double>(i) /
                                 static_cast<double>(kSegmentCostSamples);
                const double lPrime = polynomial.Evaluate(1, s);
                const double lDoublePrime = polynomial.Evaluate(2, s);
                cost += wLPrime * lPrime * lPrime +
                        wLDoublePrime * lDoublePrime * lDoublePrime;
            }

            return cost * deltaS / static_cast<double>(kSegmentCostSamples);
        }

        bool ValidConfig(const DpPlannerConfig &config)
        {
            return config.s_step > kEpsilon &&
                   config.s_step_count > 0 &&
                   config.l_step > kEpsilon &&
                   config.left_l_step_count >= 0 &&
                   config.right_l_step_count >= 0 &&
                   ValidStaticCollisionCostConfig(config.collision);
        }

    } // namespace

    DpPlanner::DpPlanner(const DpPlannerConfig &config)
        : config_(config)
    {
    }

    const DpPlannerConfig &DpPlanner::config() const
    {
        return config_;
    }

    void DpPlanner::SetConfig(const DpPlannerConfig &config)
    {
        config_ = config;
    }

    bool DpPlanner::Plan(const StartPointFrenetState &start,
                         const std::vector<StaticObsFrenetState> &obstacles,
                         DpPlannerResult *result,
                         double max_s) const
    {
        if (result == nullptr)

            return false;

        DpPlannerResult output;
        const DpPlannerConfig &config = config_;
        if (!ValidConfig(config))
        {
            output.Flag = DpPlannerFallback::Other;
            *result = std::move(output);
            return false;
        }

        const std::vector<double> sValues = BuildSValues(start.s, config, max_s);
        const std::vector<double> lSamples = BuildLSamples(start.l, config);
        if (sValues.size() < 2 || lSamples.empty())
        {

            output.Flag = DpPlannerFallback::Other;
            *result = std::move(output);
            return false;
        }

        std::vector<std::vector<DpNode>> layers(sValues.size());
        const double startCost = StaticObstacleCollisionCost(
            {start.s, start.l}, obstacles, config.collision);
        if (IsFatalCollisionCost(startCost, config.collision))
        {
            PluginLogEcho("【DpPlanner】:起点碰撞\n");
            output.Flag = DpPlannerFallback::Stop;
            *result = std::move(output);
            return false;
        }
        layers.front().push_back({{start.s,
                                   start.l,
                                   start.l_prime,
                                   start.l_double_prime},
                                  startCost,
                                  -1});
        for (std::size_t layerIndex = 1; layerIndex < sValues.size(); ++layerIndex)
        {
            layers[layerIndex].reserve(lSamples.size());
            for (double l : lSamples)
                layers[layerIndex].push_back({{sValues[layerIndex], l, 0.0, 0.0},
                                              std::numeric_limits<double>::infinity(),
                                              -1});
        }

        for (std::size_t layerIndex = 1; layerIndex < layers.size(); ++layerIndex)
        {
            const double deltaS = sValues[layerIndex] - sValues[layerIndex - 1];
            std::vector<DpNode> &currentLayer = layers[layerIndex];
            const std::vector<DpNode> &previousLayer = layers[layerIndex - 1];

            for (std::size_t currentIndex = 0; currentIndex < currentLayer.size(); ++currentIndex)
            {
                DpNode &current = currentLayer[currentIndex];
                const double collisionCost = StaticObstacleCollisionCost(
                    {current.point.s, current.point.l}, obstacles, config.collision);
                if (IsFatalCollisionCost(collisionCost, config.collision))
                    continue;

                const double weightedCollisionCost =
                    std::max(0.0, config.weight_collision) * collisionCost;
                const double referenceCost = ReferenceNodeCost(current.point.l, config);

                for (std::size_t previousIndex = 0; previousIndex < previousLayer.size(); ++previousIndex)
                {
                    const DpNode &previous = previousLayer[previousIndex];
                    if (!std::isfinite(previous.cost))
                        continue;

                    const QuinticBoundary startBoundary{
                        previous.point.l,
                        layerIndex == 1 ? start.l_prime : 0.0,
                        layerIndex == 1 ? start.l_double_prime : 0.0,
                    };
                    const QuinticBoundary endBoundary{current.point.l, 0.0, 0.0};
                    const double smoothCost =
                        SmoothSegmentCost(deltaS, startBoundary, endBoundary, config);
                    if (!std::isfinite(smoothCost))
                        continue;

                    const double totalCost =
                        previous.cost + smoothCost + referenceCost + weightedCollisionCost;
                    if (totalCost < current.cost)
                    {
                        current.cost = totalCost;
                        current.previous_index = static_cast<int>(previousIndex);
                    }
                }
            }
        }

        const std::vector<DpNode> &finalLayer = layers.back();
        int bestIndex = -1;
        double bestCost = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < finalLayer.size(); ++i)
        {
            if (finalLayer[i].cost < bestCost)
            {
                bestCost = finalLayer[i].cost;
                bestIndex = static_cast<int>(i);
            }
        }

        if (bestIndex < 0 || !std::isfinite(bestCost))
        {
            output.Flag = DpPlannerFallback::Stop;
            output.total_cost = std::numeric_limits<double>::infinity();
            *result = output;
            return false;
        }

        std::vector<DpPathPoint> reversedPath;
        int index = bestIndex;
        for (std::size_t layerIndex = layers.size() - 1; layerIndex > 0; --layerIndex)
        {
            const DpNode &node = layers[layerIndex][static_cast<std::size_t>(index)];
            reversedPath.push_back(node.point);
            index = node.previous_index;
        }
        reversedPath.push_back({start.s, start.l, start.l_prime, start.l_double_prime});
        std::reverse(reversedPath.begin(), reversedPath.end());

        output.total_cost = bestCost;
        output.Flag = DpPlannerFallback::Success;
        output.path = std::move(reversedPath);
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
