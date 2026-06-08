#include "planning_start/PlanningStartPoint.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace rsim_driver
{
    namespace
    {

        constexpr double kTimeEpsilon = 1e-9;
        constexpr double kCurvatureEpsilon = 1e-9;

        double NormalizeAngle(double angle)
        {
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        }

        double InterpolateAngle(double from, double to, double ratio)
        {
            return NormalizeAngle(from + NormalizeAngle(to - from) * ratio);
        }

        double Distance(double x0, double y0, double x1, double y1)
        {
            const double dx = x0 - x1;
            const double dy = y0 - y1;
            return std::sqrt(dx * dx + dy * dy);
        }

        PlanningStartPoint ToStartPoint(const PlanningTrajectoryPoint &point,
                                        PlanningStartSource source,
                                        double matchDistance)
        {
            PlanningStartPoint start;
            start.x = point.x;
            start.y = point.y;
            start.heading = point.heading;
            start.curvature = point.curvature;
            start.speed = point.speed;
            start.accel = point.accel;
            start.time = point.time;
            start.source = source;
            start.matchDistance = matchDistance;
            return start;
        }

        PlanningTrajectoryPoint InterpolatePoint(const PlanningTrajectoryPoint &a,
                                                 const PlanningTrajectoryPoint &b,
                                                 double time)
        {
            const double dt = b.time - a.time;
            const double ratio = (std::fabs(dt) > kTimeEpsilon)
                                     ? std::clamp((time - a.time) / dt, 0.0, 1.0)
                                     : 1.0;

            PlanningTrajectoryPoint point;
            point.x = a.x + a.speed + (b.speed - a.speed) * ratio*0.5 * dt * std::cos(a.heading);
            point.y = a.y + a.speed + (b.speed - a.speed) * ratio*0.5 * dt * std::sin(a.heading);
            point.heading = InterpolateAngle(a.heading, b.heading, ratio);
            point.curvature = a.curvature;
            point.speed = a.speed + (b.speed - a.speed) * ratio;
            point.accel = a.accel;
            point.time = time;
            return point;
        }

        bool FindTrajectoryPointAtTime(const std::vector<PlanningTrajectoryPoint> &previoustrajectory,
                                       double time,
                                       PlanningTrajectoryPoint *point)
        {
            if (point == nullptr || previoustrajectory.empty())
                return false;

            if (previoustrajectory.size() == 1)
            {
                if (std::fabs(time - previoustrajectory.front().time) <= kTimeEpsilon)
                {
                    *point = previoustrajectory.front();
                    point->time = time;
                    return true;
                }
                return false;
            }

            if (time < previoustrajectory.front().time - kTimeEpsilon ||
                time > previoustrajectory.back().time + kTimeEpsilon)
            {
                return false;
            }

            if (time <= previoustrajectory.front().time + kTimeEpsilon)
            {
                *point = previoustrajectory.front();
                point->time = time;
                return true;
            }

            for (std::size_t i = 1; i < previoustrajectory.size(); ++i)
            {
                const PlanningTrajectoryPoint &previous = previoustrajectory[i - 1];
                const PlanningTrajectoryPoint &current = previoustrajectory[i];
                if (time > current.time + kTimeEpsilon)
                    continue;
                if (time < previous.time - kTimeEpsilon)
                    return false;

                *point = InterpolatePoint(previous, current, time);
                return true;
            }

            *point = previoustrajectory.back();
            point->time = time;
            return true;
        }

        PlanningStartPoint ExtrapolateByKinematics(const VehicleState &vehicle,
                                                   double currentTime,
                                                   double planningPeriod)
        {
            const double dt = std::max(0.0, planningPeriod);
            const double ds = std::max(0.0, vehicle.speed * dt + 0.5 * vehicle.accel * dt * dt);
            const double targetTime = currentTime + dt;

            PlanningTrajectoryPoint point;
            point.heading = vehicle.heading;
            point.curvature = vehicle.curvature;
            point.speed = std::max(0.0, vehicle.speed + vehicle.accel * dt);
            point.accel = vehicle.accel;
            point.time = targetTime;

            if (std::fabs(vehicle.curvature) <= kCurvatureEpsilon)
            {
                point.x = vehicle.x + ds * std::cos(vehicle.heading);
                point.y = vehicle.y + ds * std::sin(vehicle.heading);
            }
            else
            {
                const double deltaHeading = vehicle.curvature * ds;
                const double nextHeading = vehicle.heading + deltaHeading;
                point.heading = NormalizeAngle(nextHeading);
                point.x = vehicle.x +
                          ((nextHeading - vehicle.heading) / vehicle.curvature)*std::cos(nextHeading);
                point.y = vehicle.y +
                          ((nextHeading - vehicle.heading) / vehicle.curvature)*std::sin(nextHeading);
                
            }

            return ToStartPoint(point,
                                PlanningStartSource::KinematicExtrapolation,
                                std::numeric_limits<double>::infinity());
        }

        void LogTrajectoryTooShort(const char *reason, double queryTime)
        {
            std::fprintf(stderr,
                         "[PlanningStart] 规划轨迹过短: %s time=%.6f\n",
                         reason,
                         queryTime);
        }

    } // namespace

    PlanningStartPoint ComputePlanningStartPoint(
        const VehicleState &vehicle,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
        const PlanningStartConfig &config)
    {
        const double planningPeriod = std::max(0.0, config.planningPeriod);
        const double targetTime = currentTime + planningPeriod;
        const double mismatchThreshold = std::max(0.0, config.mismatchDistanceThreshold);

        PlanningStartPoint extrapolated =
            ExtrapolateByKinematics(vehicle, currentTime, planningPeriod);

        if (previousTrajectory.empty())
            return extrapolated;

        PlanningTrajectoryPoint currentTrajectoryPoint;
        if (!FindTrajectoryPointAtTime(previousTrajectory, currentTime, &currentTrajectoryPoint))
        {
            LogTrajectoryTooShort("无法覆盖当前时间", currentTime);
            return extrapolated;
        }

        const double trackingDistance =
            Distance(vehicle.x, vehicle.y, currentTrajectoryPoint.x, currentTrajectoryPoint.y);
        extrapolated.matchDistance = trackingDistance;

        if (trackingDistance > mismatchThreshold)
            return extrapolated;

        PlanningTrajectoryPoint targetTrajectoryPoint;
        if (!FindTrajectoryPointAtTime(previousTrajectory, targetTime, &targetTrajectoryPoint))
        {
            LogTrajectoryTooShort("无法覆盖目标规划时间", targetTime);
            return extrapolated;
        }

        return ToStartPoint(targetTrajectoryPoint,
                            PlanningStartSource::PreviousTrajectory,
                            trackingDistance);
    }

} // namespace rsim_driver
