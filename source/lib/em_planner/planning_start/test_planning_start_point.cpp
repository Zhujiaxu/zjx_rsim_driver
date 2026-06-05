#include "planning_start/PlanningStartPoint.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::VehicleState MakeVehicle()
{
    rsim_driver::VehicleState vehicle;
    vehicle.x = 0.0;
    vehicle.y = 0.0;
    vehicle.heading = 0.0;
    vehicle.curvature = 0.0;
    vehicle.speed = 10.0;
    vehicle.accel = 0.0;
    return vehicle;
}

rsim_driver::PlanningTrajectoryPoint MakePoint(double time,
                                               double x,
                                               double y = 0.0,
                                               double heading = 0.0,
                                               double curvature = 0.0,
                                               double speed = 10.0,
                                               double accel = 0.0)
{
    rsim_driver::PlanningTrajectoryPoint point;
    point.x = x;
    point.y = y;
    point.heading = heading;
    point.curvature = curvature;
    point.speed = speed;
    point.accel = accel;
    point.time = time;
    return point;
}

}  // namespace

int main()
{
    rsim_driver::PlanningStartConfig config;
    config.planningPeriod = 0.01;
    config.mismatchDistanceThreshold = 0.3;

    const double currentTime = 1.0;
    const rsim_driver::VehicleState vehicle = MakeVehicle();

    rsim_driver::PlanningStartPoint start =
        rsim_driver::ComputePlanningStartPoint(vehicle, currentTime, {}, config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "empty previous trajectory should use extrapolated point"))
        return 1;
    if (!Require(Near(start.x, 0.1, 1e-9) && Near(start.y, 0.0, 1e-9),
                 "straight extrapolation should advance by speed * planning period"))
        return 1;
    if (!Require(Near(start.time, 1.01, 1e-9),
                 "extrapolated point should use target planning time"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> tooShortForCurrent = {
        MakePoint(0.50, -5.0),
        MakePoint(0.90, -1.0),
    };
    start = rsim_driver::ComputePlanningStartPoint(vehicle,
                                                   currentTime,
                                                   tooShortForCurrent,
                                                   config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "trajectory that does not cover current time should use extrapolated point"))
        return 1;
    if (!Require(Near(start.x, 0.1, 1e-9),
                 "current-time short trajectory fallback should be extrapolated"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> mismatchedTrajectory = {
        MakePoint(1.00, 1.0),
        MakePoint(1.02, 1.2),
    };
    start = rsim_driver::ComputePlanningStartPoint(vehicle,
                                                   currentTime,
                                                   mismatchedTrajectory,
                                                   config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "tracking mismatch should use extrapolated point"))
        return 1;
    if (!Require(Near(start.matchDistance, 1.0, 1e-9),
                 "tracking mismatch distance should be reported"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> tooShortForTarget = {
        MakePoint(1.000, 0.0),
        MakePoint(1.005, 0.05),
    };
    start = rsim_driver::ComputePlanningStartPoint(vehicle,
                                                   currentTime,
                                                   tooShortForTarget,
                                                   config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "trajectory that does not cover target time should use extrapolated point"))
        return 1;
    if (!Require(Near(start.matchDistance, 0.0, 1e-9),
                 "target-time short trajectory should keep current tracking distance"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> reusableTrajectory = {
        MakePoint(1.00, 0.0, 0.0, 0.00, 0.00, 8.0, 0.5),
        MakePoint(1.02, 0.2, 0.0, 0.20, 0.02, 12.0, 1.5),
    };
    start = rsim_driver::ComputePlanningStartPoint(vehicle,
                                                   currentTime,
                                                   reusableTrajectory,
                                                   config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::PreviousTrajectory,
                 "well-tracked trajectory should reuse previous trajectory target-time point"))
        return 1;
    if (!Require(Near(start.x, 0.1, 1e-9) &&
                 Near(start.heading, 0.1, 1e-9) &&
                 Near(start.curvature, 0.01, 1e-9) &&
                 Near(start.speed, 10.0, 1e-9) &&
                 Near(start.accel, 1.0, 1e-9) &&
                 Near(start.time, 1.01, 1e-9),
                 "previous trajectory point should be interpolated at target time"))
        return 1;

    rsim_driver::VehicleState curvedVehicle = MakeVehicle();
    curvedVehicle.curvature = 0.1;
    start = rsim_driver::ComputePlanningStartPoint(curvedVehicle, currentTime, {}, config);
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "curved first planning point should use extrapolation"))
        return 1;
    if (!Require(Near(start.heading, 0.01, 1e-9),
                 "curved extrapolation heading should update by curvature * ds"))
        return 1;
    if (!Require(Near(start.x, std::sin(0.01) / 0.1, 1e-9) &&
                 Near(start.y, (1.0 - std::cos(0.01)) / 0.1, 1e-9),
                 "curved extrapolation should follow constant-curvature arc"))
        return 1;

    std::fprintf(stderr, "PASS planning_start_point smoke\n");
    return 0;
}
