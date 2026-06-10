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
constexpr int kSegmentCostSamples = 5;

struct DpNode
{
    DpPathPoint point;
    double cost = std::numeric_limits<double>::infinity();
    int previous_index = -1;
};

bool Near(double a, double b)
{
    return std::fabs(a - b) <= 1e-8;
}

void AppendUnique(std::vector<double>* values, double value)
{
    if (values == nullptr || !std::isfinite(value))
        return;
    for (double existing : *values)
    {
        if (Near(existing, value))
            return;
    }
    values->push_back(value);
}

std::vector<double> BuildSValues(double startS, const DpPlannerConfig& config)
{
    std::vector<double> values;
    if (config.s_step <= kEpsilon || config.total_length <= kEpsilon)
        return values;

    const double endS = startS + config.total_length;
    values.push_back(startS);
    for (double s = startS + config.s_step; s < endS - 1e-8; s += config.s_step)
        values.push_back(s);
    AppendUnique(&values, endS);
    return values;
}

std::vector<double> BuildLSamples(const DpPlannerConfig& config)
{
    std::vector<double> samples;
    if (config.l_step <= kEpsilon || config.left_width < -kEpsilon ||
        config.right_width < -kEpsilon)
    {
        return samples;
    }

    samples.push_back(0.0);
    for (double l = config.l_step; l <= config.left_width + 1e-8; l += config.l_step)
        AppendUnique(&samples, l);
    AppendUnique(&samples, std::max(0.0, config.left_width));

    for (double l = -config.l_step; l >= -config.right_width - 1e-8; l -= config.l_step)
        AppendUnique(&samples, l);
    AppendUnique(&samples, -std::max(0.0, config.right_width));

    std::sort(samples.begin(), samples.end());
    return samples;
}

std::vector<SlPoint> ToSlPoints(
    const std::vector<StaticFrenetObstacle>& obstacles)
{
    std::vector<SlPoint> points;
    points.reserve(obstacles.size());
    for (const StaticFrenetObstacle& obstacle : obstacles)
        points.push_back({obstacle.s, obstacle.l});
    return points;
}

bool IsFatalCollisionCost(double cost, const CollisionCostConfig& config)
{
    if (!std::isfinite(cost))
        return true;
    if (std::isfinite(config.infinity_cost) && cost == config.infinity_cost)
        return true;
    return false;
}

double ReferenceNodeCost(double l, const DpPlannerConfig& config)
{
    return std::max(0.0, config.weight_ref_l) * l * l;
}

double SmoothSegmentCost(double deltaS,
                         const QuinticBoundary& start,
                         const QuinticBoundary& end,
                         const DpPlannerConfig& config)
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

bool ValidConfig(const DpPlannerConfig& config)
{
    return config.s_step > kEpsilon &&
           config.total_length > kEpsilon &&
           config.l_step > kEpsilon &&
           config.left_width >= -kEpsilon &&
           config.right_width >= -kEpsilon;
}

}  // namespace

bool Plan(const CartesianFrenetState& start,
          const std::vector<StaticFrenetObstacle>& obstacles,
          const DpPlannerConfig& config,
          DpPlannerResult* result)
{
    if (result == nullptr)
        return false;

    DpPlannerResult output;
    if (!ValidConfig(config))
    {
        *result = output;
        return false;
    }

    const std::vector<double> sValues = BuildSValues(start.s, config);
    const std::vector<double> lSamples = BuildLSamples(config);
    if (sValues.size() < 2 || lSamples.empty())
    {
        *result = output;
        return false;
    }

    std::vector<std::vector<DpNode>> layers(sValues.size());
    const double startCost = ReferenceNodeCost(start.l, config);
    layers.front().push_back({{start.s, start.l}, startCost, -1});
    for (std::size_t layerIndex = 1; layerIndex < sValues.size(); ++layerIndex)
    {
        layers[layerIndex].reserve(lSamples.size());
        for (double l : lSamples)
            layers[layerIndex].push_back({{sValues[layerIndex], l},
                                          std::numeric_limits<double>::infinity(),
                                          -1});
    }

    const std::vector<SlPoint> obstaclePoints = ToSlPoints(obstacles);
    for (std::size_t layerIndex = 1; layerIndex < layers.size(); ++layerIndex)
    {
        const double deltaS = sValues[layerIndex] - sValues[layerIndex - 1];
        std::vector<DpNode>& currentLayer = layers[layerIndex];
        const std::vector<DpNode>& previousLayer = layers[layerIndex - 1];

        for (std::size_t currentIndex = 0; currentIndex < currentLayer.size(); ++currentIndex)
        {
            DpNode& current = currentLayer[currentIndex];
            const double collisionCost = ObstacleCollisionCost(
                {current.point.s, current.point.l}, obstaclePoints, config.collision);
            if (IsFatalCollisionCost(collisionCost, config.collision))
                continue;

            const double weightedCollisionCost =
                std::max(0.0, config.weight_collision) * collisionCost;
            const double referenceCost = ReferenceNodeCost(current.point.l, config);

            for (std::size_t previousIndex = 0; previousIndex < previousLayer.size(); ++previousIndex)
            {
                const DpNode& previous = previousLayer[previousIndex];
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

    const std::vector<DpNode>& finalLayer = layers.back();
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
        *result = output;
        return false;
    }

    std::vector<DpPathPoint> reversedPath;
    int index = bestIndex;
    for (std::size_t layerIndex = layers.size() - 1; layerIndex > 0; --layerIndex)
    {
        const DpNode& node = layers[layerIndex][static_cast<std::size_t>(index)];
        reversedPath.push_back(node.point);
        index = node.previous_index;
        if (index < 0 && layerIndex > 1)
        {
            *result = output;
            return false;
        }
    }
    reversedPath.push_back({start.s, start.l});
    std::reverse(reversedPath.begin(), reversedPath.end());

    output.dpsuccess = true;
    output.total_cost = bestCost;
    output.path = std::move(reversedPath);
    *result = std::move(output);
    return true;
}

}  // namespace rsim_driver
