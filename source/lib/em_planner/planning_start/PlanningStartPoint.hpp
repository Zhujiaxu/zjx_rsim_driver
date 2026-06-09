#pragma once

#include <vector>

namespace rsim_driver
{

enum class PlanningStartSource
{
    KinematicExtrapolation,
    PreviousTrajectory
};

struct VehicleState
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
};

struct PlanningTrajectoryPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double curvature = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double time = 0.0;
};

struct PlanningStartPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double time = 0.0;
    PlanningStartSource source = PlanningStartSource::KinematicExtrapolation;
    double matchDistance = 0.0;
};

struct PlanningStartConfig
{
    double planningPeriod = 0.01;
    double mismatchDistanceThreshold = 0.3;
};

struct PlanningStartResult
{
    PlanningStartPoint start_point;
    double start_curvature = 0.0;
    std::vector<PlanningTrajectoryPoint> stitching_trajectory;
};

PlanningStartResult ComputePlanningStartResult(
    const VehicleState& vehicle,
    double currentTime,
    const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
    const PlanningStartConfig& config = {});

}  // namespace rsim_driver
