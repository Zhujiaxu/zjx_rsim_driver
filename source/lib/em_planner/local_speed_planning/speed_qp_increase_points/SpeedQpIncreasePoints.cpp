#include "SpeedQpIncreasePoints.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace rsim_driver
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPostSolveTolerance = 1e-6;
constexpr double kContinuityTolerance = 1e-6;

bool IsFinite(const DynamicPlanSpeedPoint& point)
{
    return std::isfinite(point.t) && std::isfinite(point.s) &&
           std::isfinite(point.v) && std::isfinite(point.a);
}

bool CanonicalizeNonNegative(double* value)
{
    if (value == nullptr || !std::isfinite(*value) ||
        *value < -kPostSolveTolerance)
    {
        return false;
    }
    if (*value < 0.0)
        *value = 0.0;
    return true;
}

bool CanonicalizePoint(DynamicPlanSpeedPoint* point)
{
    return point != nullptr && IsFinite(*point) &&
           CanonicalizeNonNegative(&point->s) &&
           CanonicalizeNonNegative(&point->v);
}

bool Near(double actual, double expected)
{
    return std::fabs(actual - expected) <= kContinuityTolerance;
}

DynamicPlanSpeedPoint InterpolateConstantJerk(
    const DynamicPlanSpeedPoint& start,
    const DynamicPlanSpeedPoint& end,
    double time)
{
    const double deltaT = end.t - start.t;
    const double offset = time - start.t;
    const double jerk = (end.a - start.a) / deltaT;
    const double offsetSquared = offset * offset;

    DynamicPlanSpeedPoint point;
    point.t = time;
    point.s = start.s + start.v * offset +
              0.5 * start.a * offsetSquared +
              jerk * offsetSquared * offset / 6.0;
    point.v = start.v + start.a * offset +
              0.5 * jerk * offsetSquared;
    point.a = start.a + jerk * offset;
    return point;
}

bool IsKinematicallyContinuous(const DynamicPlanSpeedPoint& start,
                               const DynamicPlanSpeedPoint& end)
{
    const DynamicPlanSpeedPoint expected =
        InterpolateConstantJerk(start, end, end.t);
    return Near(end.s, expected.s) && Near(end.v, expected.v);
}

bool IsForwardFrom(const DynamicPlanSpeedPoint& previous,
                   const DynamicPlanSpeedPoint& current)
{
    return current.t - previous.t > kEpsilon &&
           current.s + kEpsilon >= previous.s;
}

}  // namespace

QpSpeedIncreasePoints::QpSpeedIncreasePoints(
    QpSpeedIncreasePointsConfig config)
    : config_(config)
{
}

void QpSpeedIncreasePoints::SetConfig(
    const QpSpeedIncreasePointsConfig& config)
{
    config_ = config;
}

const QpSpeedIncreasePointsConfig& QpSpeedIncreasePoints::config() const
{
    return config_;
}

bool QpSpeedIncreasePoints::increasepoints(
    const QpSpeedOptimizerResult & speed_qp_result,
    std::vector<DynamicPlanSpeedPoint>* newqppointspath) const
{
    if (newqppointspath == nullptr)
        return false;

    newqppointspath->clear();
    if (config_.count < 1 || speed_qp_result.stpoints.size() < 2)
        return false;

    std::vector<DynamicPlanSpeedPoint> normalizedPath = speed_qp_result.stpoints;
    for (std::size_t i = 0; i < normalizedPath.size(); ++i)
    {
        if (!CanonicalizePoint(&normalizedPath[i]))
            return false;
        if (i > 0 && !IsForwardFrom(normalizedPath[i - 1], normalizedPath[i]))
            return false;
    }

    const std::size_t subdivisions =
        static_cast<std::size_t>(config_.count);
    const std::size_t intervalCount = normalizedPath.size() - 1U;
    if (intervalCount >
        (std::numeric_limits<std::size_t>::max() - 1U) / subdivisions)
    {
        return false;
    }

    std::vector<DynamicPlanSpeedPoint> output;
    output.reserve(1U + intervalCount * subdivisions);
    output.push_back(normalizedPath.front());

    for (std::size_t i = 0; i < intervalCount; ++i)
    {
        const DynamicPlanSpeedPoint& start = normalizedPath[i];
        const DynamicPlanSpeedPoint& end = normalizedPath[i + 1U];
        if (!IsKinematicallyContinuous(start, end))
            return false;

        const double deltaT = end.t - start.t;
        for (std::size_t subdivision = 1;
             subdivision <= subdivisions;
             ++subdivision)
        {
            DynamicPlanSpeedPoint point;
            if (subdivision == subdivisions)
            {
                point = end;
            }
            else
            {
                const double time =
                    start.t + deltaT * static_cast<double>(subdivision) /
                                  static_cast<double>(subdivisions);
                point = InterpolateConstantJerk(start, end, time);
                if (!CanonicalizePoint(&point))
                    return false;
            }

            if (!IsForwardFrom(output.back(), point))
                return false;
            output.push_back(point);
        }
    }

    *newqppointspath = std::move(output);
    return true;
}

}  // namespace rsim_driver
