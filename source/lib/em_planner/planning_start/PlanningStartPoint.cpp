#include "PlanningStartPoint.hpp"

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

        /*double EgoPlanarAccel(const rsim_plugin::ActorState &actor)
        {
            return std::sqrt(actor.acc_x * actor.acc_x +
                             actor.acc_y * actor.acc_y);
        }*/

        bool IsValidTrajectoryPoint(const PlanningTrajectoryPoint &point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) &&
                   std::isfinite(point.heading) &&
                   std::isfinite(point.curvature) &&
                   std::isfinite(point.speed) && point.speed >= 0.0 &&
                   std::isfinite(point.accel) && std::isfinite(point.time);
        }

        bool IsValidInput(const rsim_plugin::ActorState &ego,
                          double currentTime,
                          const std::vector<PlanningTrajectoryPoint> &trajectory,
                          const PlanningStartConfig &config)
        {
            if (!std::isfinite(ego.x) || !std::isfinite(ego.y) ||
                !std::isfinite(ego.h) || !std::isfinite(ego.speed) ||
                ego.speed < 0.0 || !std::isfinite(ego.acc_x) ||
                !std::isfinite(ego.acc_y) || !std::isfinite(currentTime) ||
                !std::isfinite(config.planningPeriod) ||
                config.planningPeriod < 0.0 ||
                !std::isfinite(config.mismatchDistanceThreshold) ||
                config.mismatchDistanceThreshold < 0.0)
            {
                return false;
            }
            for (std::size_t i = 0; i < trajectory.size(); ++i)
            {
                if (!IsValidTrajectoryPoint(trajectory[i]))
                    return false;
                if (i > 0 && trajectory[i].time - trajectory[i - 1].time <=
                                 kTimeEpsilon)
                {
                    return false;
                }
            }
            return true;
        }

        PlanningStartPoint ToStartPoint(const PlanningTrajectoryPoint &point,
                                        PlanningStartSource source,
                                        double matchDistance)
        {
            PlanningStartPoint start;
            start.startpointbasis = point;
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
            const rsim_plugin::ActorState &ego,
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
            check.match_distance = Distance(ego.x,
                                            ego.y,
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

        double EgoXAccel(const rsim_plugin::ActorState &actor)
        {
            return actor.acc_x*std::cos(actor.h) - actor.acc_y*std::sin(actor.h);
        }
        double EgoYAccel(const rsim_plugin::ActorState &actor)
        {
            return actor.acc_x*std::sin(actor.h) + actor.acc_y*std::cos(actor.h);
        }

        double EgoXspeed(const rsim_plugin::ActorState &actor)
        {
            return actor.vel_x*std::cos(actor.h) - actor.vel_y*std::sin(actor.h);
        }
        double EgoYspeed(const rsim_plugin::ActorState &actor)
        {
            return actor.vel_x*std::sin(actor.h) + actor.vel_y*std::cos(actor.h);
        }   
        double EgoSpeed(const rsim_plugin::ActorState &actor)
        {
            return std::sqrt(actor.vel_x*actor.vel_x + actor.vel_y*actor.vel_y);
        }
        PlanningStartPoint ExtrapolateByKinematics(const rsim_plugin::ActorState &ego,
                                                   double currentTime,
                                                   double planningPeriod)
        {
            const double dt = std::max(0.0, planningPeriod);
            const double targetTime = currentTime + dt;

            PlanningTrajectoryPoint point;
            point.x = ego.x + EgoXspeed(ego) * dt + 0.5 * EgoXAccel(ego) * dt * dt;
            point.y = ego.y + EgoYspeed(ego) * dt + 0.5 * EgoYAccel(ego) * dt * dt;
            point.heading = ego.h;
            point.curvature = 0.0;
            point.speed =std::sqrt((EgoXspeed(ego)+EgoXAccel(ego)*dt )*(EgoXspeed(ego)+EgoXAccel(ego)*dt) 
                        + (EgoYspeed(ego)+EgoYAccel(ego)*dt )*(EgoYspeed(ego)+EgoYAccel(ego)*dt)); // ego.speed + EgoPlanarAccel(ego) * dt;
            point.accel = ego.acc_x;
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

    PlanningStart::PlanningStart(const PlanningStartConfig &config)
        : planningStartPointConfig_(config)
    {
    }

    const PlanningStartConfig &PlanningStart::config() const
    {
        return planningStartPointConfig_;
    }

    void PlanningStart::SetConfig(const PlanningStartConfig &config)
    {
        planningStartPointConfig_ = config;
    }

    bool PlanningStart::Compute(
        const rsim_plugin::ActorState &ego,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint> &previousTrajectory,
        PlanningStartResult *result) const
    {
        if (result == nullptr)
            return false;
        if (!IsValidInput(ego, currentTime, previousTrajectory,
                          planningStartPointConfig_))
        {
            return false;
        }

        const double planningPeriod = planningStartPointConfig_.planningPeriod;
        const double targetTime = currentTime + planningPeriod;

        PlanningStartResult output;
        output.start_point =
            ExtrapolateByKinematics(ego, currentTime, planningPeriod);
        output.start_curvature = 0.0;
        output.stitching_trajectory.clear();
        if (previousTrajectory.empty())
        {
            *result = std::move(output);
            return true;
        }

        PreviousTrajectoryReuseCheck check = EvaluatePreviousTrajectoryReuse(
            ego, currentTime, previousTrajectory, planningStartPointConfig_);
        if (!check.has_current_point)
        {
            LogTrajectoryTooShort("无法找到当前时间点的轨迹点", currentTime);
            *result = std::move(output);
            return true;
        }
        output.start_point.matchDistance = check.match_distance;
        if (!check.reusable)
        {
            LogTrajectoryTooShort("跟踪延迟——距离过大", currentTime);
            *result = std::move(output);
            return true;
        }
        else
        {
            PlanningTrajectoryPoint startPoint;
            if (!FindTrajectoryPointAtTime(previousTrajectory, targetTime, &startPoint))
            {
                LogTrajectoryTooShort("无法找到目标时间点的轨迹点", targetTime);
                *result = std::move(output);
                return true;
            }
            output.start_point = ToStartPoint(startPoint,
                                              PlanningStartSource::PreviousTrajectory,
                                              check.match_distance);
            output.start_curvature = startPoint.curvature;
            output.stitching_trajectory = CollectStitchingTrajectory(previousTrajectory, targetTime);
        }
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
