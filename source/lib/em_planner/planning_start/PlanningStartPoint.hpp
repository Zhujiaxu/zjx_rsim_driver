#pragma once

#include "CartesianToFrenet.hpp"
#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <vector>

namespace rsim_driver
{

enum class PlanningStartSource
{
    KinematicExtrapolation,
    PreviousTrajectory
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
    PlanningTrajectoryPoint startpointbasis;
    PlanningStartSource source = PlanningStartSource::KinematicExtrapolation;
    double matchDistance = 0.0;
};

struct PlanningStartConfig
{
    double planningPeriod = 0.05;
    double mismatchDistanceThreshold = 0.3;
};

struct PlanningStartResult
{
    PlanningStartPoint start_point;
    double start_curvature = 0.0;
    std::vector<PlanningTrajectoryPoint> stitching_trajectory;
};

class PlanningStart
{
public:
    explicit PlanningStart(const PlanningStartConfig& config = {});

    const PlanningStartConfig& config() const;
    void SetConfig(const PlanningStartConfig& config);

    bool Compute(
        const rsim_plugin::ActorState& ego,
        double currentTime,
        const std::vector<PlanningTrajectoryPoint>& previousTrajectory,
        PlanningStartResult *result) const;

    template <typename RefPointT>
    bool ToFrenet(
        const PlanningStartResult& result,
        const std::vector<RefPointT>& referencePoints,
        StartPointFrenetState* frenetState) const
    {
        return CartesianToFrenet(referencePoints, result, frenetState);
    }

private:
    PlanningStartConfig planningStartPointConfig_;
};

}  // namespace rsim_driver
