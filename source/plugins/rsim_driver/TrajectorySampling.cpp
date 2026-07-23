#include "TrajectorySampling.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver::plugin_internal
{

    namespace
    {

        constexpr double kMinTimeStep = 1e-6;
        constexpr double kPi = 3.14159265358979323846;

        double NormalizeAngle(double angle)
        {
            while (angle > kPi)
                angle -= 2.0 * kPi;
            while (angle < -kPi)
                angle += 2.0 * kPi;
            return angle;
        }

        double InterpolateAngle(double from, double to, double ratio)
        {
            return NormalizeAngle(from + NormalizeAngle(to - from) * ratio);
        }

        PlanningTrajectoryPoint InterpolateTrajectoryPoint(
            const PlanningTrajectoryPoint &previous,
            const PlanningTrajectoryPoint &next,
            double time)
        {
            const double dt = next.time - previous.time;
            const double ratio = dt > kMinTimeStep
                                     ? std::clamp((time - previous.time) / dt, 0.0, 1.0)
                                     : 0.0;

            PlanningTrajectoryPoint point;
            point.x = previous.x + (next.x - previous.x) * ratio;
            point.y = previous.y + (next.y - previous.y) * ratio;
            point.heading = InterpolateAngle(previous.heading, next.heading, ratio);
            point.curvature =previous.curvature;
            point.speed = previous.speed + (next.speed - previous.speed) * ratio;
            point.accel = previous.accel;
            point.time = time;
            return point;
        }

    } // namespace

    /*bool IsValidTrajectoryPoint(const PlanningTrajectoryPoint &point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
               std::isfinite(point.heading) && std::isfinite(point.curvature) &&
               std::isfinite(point.speed) && point.speed >= 0.0 &&
               std::isfinite(point.accel) && std::isfinite(point.time);
    }
               
    for (std::size_t i = 0; i < trajectory.size(); ++i)
        {
            if (i > 0 &&
                trajectory[i].time < trajectory[i - 1].time - kMinTimeStep)
            {
                return false;
            }
        }*/  //我会在Emplanner.cpp中实现这个函数，避免重复检查

    bool FindTrajectoryPointAtTime(
        const std::vector<PlanningTrajectoryPoint> &trajectory,
        double queryTime,
        PlanningTrajectoryPoint *point,
        std::size_t *targetIndex)
    {
        if (point == nullptr || targetIndex == nullptr || trajectory.empty() ||
            !std::isfinite(queryTime))
        {
            return false;
        }

        if (trajectory.size() == 1)
        {
            if (std::fabs(queryTime - trajectory.front().time) > kMinTimeStep)
                return false;
            *point = trajectory.front();
            point->time = queryTime;
            *targetIndex = 0;
            return true;
        }

        if (queryTime < trajectory.front().time - kMinTimeStep ||
            queryTime > trajectory.back().time + kMinTimeStep)
        {
            return false;
        }

        if (queryTime <= trajectory.front().time + kMinTimeStep)
        {
            *point = trajectory.front();
            point->time = queryTime;
            *targetIndex = 0;
            return true;
        }

        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const PlanningTrajectoryPoint &previous = trajectory[i - 1];
            const PlanningTrajectoryPoint &current = trajectory[i];
            if (queryTime > current.time + kMinTimeStep)
                continue;
            if (queryTime < previous.time - kMinTimeStep)
                return false;

            *point = InterpolateTrajectoryPoint(previous, current, queryTime);
            *targetIndex = i;
            return true;
        }
        return false;
    }

} // namespace rsim_driver::plugin_internal
