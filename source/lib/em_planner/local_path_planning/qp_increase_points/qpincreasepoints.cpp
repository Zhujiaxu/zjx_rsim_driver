#include "qpincreasepoints.hpp"

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
constexpr double kContinuityTolerance = 2e-4;

bool IsFinite(const DpPathPoint& point)
{
    return std::isfinite(point.s) &&
           std::isfinite(point.l) &&
           std::isfinite(point.l_prime) &&
           std::isfinite(point.l_double_prime);
}

bool Near(double actual, double expected)
{
    return std::fabs(actual - expected) <= kContinuityTolerance;
}

bool IsTaylorContinuous(const DpPathPoint& start,
                        const DpPathPoint& end,
                        double deltaS)
{
    const double jerk =
        (end.l_double_prime - start.l_double_prime) / deltaS;
    const double expectedL =
        start.l + start.l_prime * deltaS +
        0.5 * start.l_double_prime * deltaS * deltaS +
        jerk * deltaS * deltaS * deltaS / 6.0;
    const double expectedLPrime =
        start.l_prime + start.l_double_prime * deltaS +
        0.5 * jerk * deltaS * deltaS;

    return Near(end.l, expectedL) && Near(end.l_prime, expectedLPrime);
}

}  // namespace

QpIncreasePoints::QpIncreasePoints(QpIncreasePointsConfig config)
    : config_(config)
{
}

void QpIncreasePoints::SetConfig(const QpIncreasePointsConfig& config)
{
    config_ = config;
}

const QpIncreasePointsConfig& QpIncreasePoints::config() const
{
    return config_;
}

bool QpIncreasePoints::increasepoints(
    const QpPathResult & qppath,
    QpIncreasePointsResult* newqppathresult) const
{
    if (newqppathresult == nullptr)
        return false;

    newqppathresult->localfrenetpath.clear();
    if (config_.count < 1 || qppath.localfrenetpath.size() < 2)
        return false;

    for (std::size_t i = 0; i < qppath.localfrenetpath.size(); ++i)
    {
        if (!IsFinite(qppath.localfrenetpath[i]))
            return false;
        if (i > 0 && qppath.localfrenetpath[i].s - qppath.localfrenetpath[i - 1].s <= kEpsilon)
            return false;
    }

    const std::size_t subdivisions =
        static_cast<std::size_t>(config_.count);
    const std::size_t intervalCount = qppath.localfrenetpath.size() - 1;
    if (intervalCount >
        (std::numeric_limits<std::size_t>::max() - 1U) / subdivisions)
    {
        return false;
    }

    QpIncreasePointsResult output;
    output.localfrenetpath.reserve(1U + intervalCount * subdivisions);
    output.localfrenetpath.push_back(qppath.localfrenetpath.front());

    for (std::size_t i = 0; i < intervalCount; ++i)
    {
        const DpPathPoint& start = qppath.localfrenetpath[i];
        const DpPathPoint& end = qppath.localfrenetpath[i + 1];
        const double deltaS = end.s - start.s;
        if (!IsTaylorContinuous(start, end, deltaS))
            return false;

        const double jerk =
            (end.l_double_prime - start.l_double_prime) / deltaS;
        for (std::size_t subdivision = 1;
             subdivision <= subdivisions;
             ++subdivision)
        {
            if (subdivision == subdivisions)
            {
                output.localfrenetpath.push_back(end);
                continue;
            }

            const double offset =
                deltaS * static_cast<double>(subdivision) /
                static_cast<double>(subdivisions);
            const double offsetSquared = offset * offset;
            const double offsetCubed = offsetSquared * offset;
            DpPathPoint point;
            point.s = start.s + offset;
            point.l = start.l + start.l_prime * offset +
                      0.5 * start.l_double_prime * offsetSquared +
                      jerk * offsetCubed / 6.0;
            point.l_prime = start.l_prime + start.l_double_prime * offset +
                            0.5 * jerk * offsetSquared;
            point.l_double_prime = start.l_double_prime + jerk * offset;
            if (!IsFinite(point))
                return false;
            output.localfrenetpath.push_back(point);
        }
    }

    newqppathresult->localfrenetpath = std::move(output.localfrenetpath);
    return true;
}

}  // namespace rsim_driver
