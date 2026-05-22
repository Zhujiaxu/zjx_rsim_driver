/*
 * RSimDriver — shared type definitions
 * All sub-modules (Sampling, Perception, TrafficRule) share these types.
 * No esmini dependencies. Header-only.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace rsim_driver
{
    // Sentinel large value (replaces esmini CommonMini::LARGE_NUMBER).
    inline constexpr double LARGE_NUMBER = 1e30;

    struct LeadVehicleInfo
    {
        bool   exists      = false;
        int    objectIndex = -1;
        double gap         = LARGE_NUMBER;  // longitudinal gap (m)
        double speed       = 0.0;           // leader speed (m/s)
        double lateralDist = 0.0;           // lateral distance (m)
    };

    struct SurroundingVehicles
    {
        LeadVehicleInfo front;
        LeadVehicleInfo leftFront, leftRear;
        LeadVehicleInfo rightFront, rightRear;
    };

    struct Obstacle
    {
        int    objectIndex = -1;
        double s           = 0.0;  // longitudinal position in the planner reference frame
        double d           = 0.0;  // lateral offset in the planner reference frame
        double speed       = 0.0;
        double length      = 0.0;
        double width       = 0.0;
    };

    struct LateralIntent
    {
        enum Type
        {
            KEEP_LANE,
            CHANGE_LEFT,
            CHANGE_RIGHT,
            NUDGE
        };
        Type   type          = KEEP_LANE;
        int    targetLaneId  = 0;
        double targetD       = 0.0;
        double lateralOffset = 0.0;
    };

    enum class DrivingState
    {
        FREE_DRIVE,
        CAR_FOLLOW,
        LANE_CHANGING,
        STOPPING,
        STOPPED
    };

    enum class LaneChangeMode
    {
        AUTO,
        OFF
    };

    inline LaneChangeMode ParseLaneChangeMode(const std::string& s)
    {
        if (s == "off")
            return LaneChangeMode::OFF;
        return LaneChangeMode::AUTO;
    }

    // Route waypoint (own struct; no rsim/rpc dependency).
    struct RouteWaypoint
    {
        int64_t trackId = 0;
        int     laneId  = 0;
        double  s       = 0.0;
        double  t       = 0.0;
        double  x       = 0.0;
        double  y       = 0.0;
        double  z       = 0.0;
        double  heading = 0.0;
        double  refS    = 0.0;  // cumulative arc length from route start
    };

    // Frenet state for trajectory planning.
    struct FrenetState
    {
        double s    = 0.0;
        double s_d  = 0.0;
        double s_dd = 0.0;
        double d    = 0.0;
        double d_d  = 0.0;
        double d_dd = 0.0;
    };

    // Quintic/quartic polynomial trajectory in Frenet coordinates.
    struct FrenetTrajectory
    {
        double sCoeffs[5] = {};
        double dCoeffs[6] = {};
        double T          = 0.0;
        double cost       = LARGE_NUMBER;
        bool   valid      = false;

        inline double EvalS(double t) const
        {
            return sCoeffs[0] + t * (sCoeffs[1] + t * (sCoeffs[2] + t * (sCoeffs[3] + t * sCoeffs[4])));
        }
        inline double EvalSdot(double t) const
        {
            return sCoeffs[1] + t * (2.0 * sCoeffs[2] + t * (3.0 * sCoeffs[3] + t * 4.0 * sCoeffs[4]));
        }
        inline double EvalSddot(double t) const
        {
            return 2.0 * sCoeffs[2] + t * (6.0 * sCoeffs[3] + t * 12.0 * sCoeffs[4]);
        }
        inline double EvalD(double t) const
        {
            return dCoeffs[0] + t * (dCoeffs[1] + t * (dCoeffs[2] + t * (dCoeffs[3] + t * (dCoeffs[4] + t * dCoeffs[5]))));
        }
        inline double EvalDdot(double t) const
        {
            return dCoeffs[1] + t * (2.0 * dCoeffs[2] + t * (3.0 * dCoeffs[3] + t * (4.0 * dCoeffs[4] + t * 5.0 * dCoeffs[5])));
        }
        inline double EvalDddot(double t) const
        {
            return 2.0 * dCoeffs[2] + t * (6.0 * dCoeffs[3] + t * (12.0 * dCoeffs[4] + t * 20.0 * dCoeffs[5]));
        }
    };

    struct TrajectoryPoint
    {
        double t         = 0.0;
        double s         = 0.0;
        double d         = 0.0;
        double x         = 0.0;
        double y         = 0.0;
        double heading   = 0.0;
        double speed     = 0.0;
        double curvature = 0.0;
    };

    // Sampled Frenet trajectory. Planner libraries fill s/d/speed/heading/curvature;
    // adapter layers may populate x/y after map projection when needed.
    struct PlannedTrajectory
    {
        static constexpr int MAX_POINTS = 50;
        TrajectoryPoint points[MAX_POINTS];
        int    numPoints = 0;
        double startTime = 0.0;
    };

    // Helpers (replace esmini CommonMini equivalents).
    inline double GetAbsAngleDifference(double a, double b)
    {
        double diff = std::fmod(a - b, 2.0 * M_PI);
        if (diff > M_PI)  diff -= 2.0 * M_PI;
        if (diff < -M_PI) diff += 2.0 * M_PI;
        return std::fabs(diff);
    }

}  // namespace rsim_driver
