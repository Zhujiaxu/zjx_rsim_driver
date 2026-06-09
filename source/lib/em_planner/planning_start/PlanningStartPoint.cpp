#include "planning_start/PlanningStartPoint.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>

namespace rsim_driver
{
    namespace
    {

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTimeEpsilon = 1e-9;
        constexpr std::size_t kMaxStitchingTrajectoryPoints = 30;

        struct PreviousTrajectoryReuseCheck
        {
            bool has_current_point = false;
            bool reusable = false;
            double match_distance = std::numeric_limits<double>::infinity();
        };

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
            point.x = a.x + (b.x - a.x) * ratio;
            point.y = a.y + (b.y - a.y) * ratio;
            point.heading = InterpolateAngle(a.heading, b.heading, ratio);
            point.curvature = a.curvature + (b.curvature - a.curvature) * ratio;
            point.speed = a.speed + (b.speed - a.speed) * ratio;
            point.accel = a.accel + (b.accel - a.accel) * ratio;
            point.time = time;
            return point;
        }

        bool FindTrajectoryPointAtTime(
            const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
            double curtime,
            PlanningTrajectoryPoint *point)
        {
            if (point == nullptr || previousTrajectory.empty())
                return false;

            if (previousTrajectory.size() == 1)
            {
                if (std::fabs(curtime - previousTrajectory.front().time) <= kTimeEpsilon)
                {
                    *point = previousTrajectory.front();
                    point->time = curtime;
                    return true;
                }
                return false;
            }

            if (curtime < previousTrajectory.front().time - kTimeEpsilon ||
                curtime > previousTrajectory.back().time + kTimeEpsilon)
            {
                return false;
            }

            if (curtime <= previousTrajectory.front().time + kTimeEpsilon)
            {
                *point = previousTrajectory.front();
                point->time = curtime;
                return true;
            }

            for (std::size_t i = 1; i < previousTrajectory.size(); ++i)
            {
                const PlanningTrajectoryPoint &previous = previousTrajectory[i - 1];
                const PlanningTrajectoryPoint &current = previousTrajectory[i];
                if (curtime > current.time + kTimeEpsilon)
                    continue;
                if (curtime < previous.time - kTimeEpsilon)
                    return false;

                *point = InterpolatePoint(previous, current, curtime);
                return true;
            }

            *point = previousTrajectory.back();
            point->time = curtime;
            return true;
        }

        PreviousTrajectoryReuseCheck EvaluatePreviousTrajectoryReuse(
            const VehicleState &vehicle,
            double currentTime,
            const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
            const PlanningStartConfig &config)
        {
            PreviousTrajectoryReuseCheck check;
            PlanningTrajectoryPoint currentTrajectoryPoint;
            if (!FindTrajectoryPointAtTime(previousTrajectory,
                                           currentTime,
                                           &currentTrajectoryPoint))
            {
                return check;
            }

            check.has_current_point = true;
            check.match_distance = Distance(vehicle.x,
                                            vehicle.y,
                                            currentTrajectoryPoint.x,
                                            currentTrajectoryPoint.y);
            check.reusable =
                check.match_distance <= std::max(0.0, config.mismatchDistanceThreshold);
            return check;
        }

        std::vector<PlanningTrajectoryPoint> CollectStitchingTrajectory(
            const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
            double targetTime)
        {
            std::vector<PlanningTrajectoryPoint> stitching;
            stitching.reserve(std::min(kMaxStitchingTrajectoryPoints,
                                       previousTrajectory.size()));

            for (auto iter = previousTrajectory.rbegin();
                 iter != previousTrajectory.rend() &&
                 stitching.size() < kMaxStitchingTrajectoryPoints;
                 ++iter)
            {
                if (iter->time >= targetTime - kTimeEpsilon)
                    continue;

                stitching.push_back(*iter);
            }

            std::reverse(stitching.begin(), stitching.end());

            return stitching;
        }

        PlanningStartPoint ExtrapolateByKinematics(const VehicleState &vehicle,
                                                   double currentTime,
                                                   double planningPeriod)
        {
            const double dt = std::max(0.0, planningPeriod);
            const double ds =
                std::max(0.0, vehicle.speed * dt + 0.5 * vehicle.accel * dt * dt);
            const double targetTime = currentTime + dt;

            PlanningTrajectoryPoint point;
            point.x = vehicle.x + ds * std::cos(vehicle.heading);
            point.y = vehicle.y + ds * std::sin(vehicle.heading);
            point.heading = vehicle.heading;
            point.speed = std::max(0.0, vehicle.speed + vehicle.accel * dt);
            point.accel = vehicle.accel;
            point.time = targetTime;

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

    PlanningStartResult ComputePlanningStartResult(
        const VehicleState &vehicle,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
        const PlanningStartConfig &config)
    {
        const double planningPeriod = std::max(0.0, config.planningPeriod);
        const double targetTime = currentTime + planningPeriod;

        PlanningStartResult result;
        result.start_point =
            ExtrapolateByKinematics(vehicle, currentTime, planningPeriod);
        result.start_curvature = 0.0;
        result.stitching_trajectory.clear();
        if (previousTrajectory.empty())
            return result;

        PreviousTrajectoryReuseCheck check = EvaluatePreviousTrajectoryReuse(
            vehicle, currentTime, previousTrajectory, config);
        if (!check.has_current_point)
        {
            LogTrajectoryTooShort("无法找到当前时间点的轨迹点", currentTime);
            return result;
        }
        result.start_point.matchDistance = check.match_distance;
        if (!check.reusable)
        {
            LogTrajectoryTooShort("跟踪延迟——距离过大", currentTime);
            return result;
        }
        else
        {
            PlanningTrajectoryPoint startPoint;
            if (!FindTrajectoryPointAtTime(previousTrajectory, targetTime, &startPoint))
            {
                LogTrajectoryTooShort("无法找到目标时间点的轨迹点", targetTime);
                return result;
            }
            result.start_point = ToStartPoint(startPoint,
                                              PlanningStartSource::PreviousTrajectory,
                                              check.match_distance);
            result.start_curvature = startPoint.curvature;
            result.stitching_trajectory = CollectStitchingTrajectory(previousTrajectory, targetTime);
        }
        return result;
    }

} // namespace rsim_driver
