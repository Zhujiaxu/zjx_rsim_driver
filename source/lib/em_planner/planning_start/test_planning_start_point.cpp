#include "planning_start/PlanningStartPoint.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{

template <typename T, typename = void>
struct HasCurvature : std::false_type
{
};

template <typename T>
struct HasCurvature<T, std::void_t<decltype(std::declval<T&>().curvature)>>
    : std::true_type
{
};

static_assert(!HasCurvature<rsim_plugin::ActorState>::value,
              "ActorState should not expose curvature");
static_assert(!HasCurvature<rsim_driver::PlanningStartPoint>::value,
              "PlanningStartPoint should not carry curvature (moved to PlanningStartResult)");
static_assert(HasCurvature<rsim_driver::PlanningTrajectoryPoint>::value,
              "PlanningTrajectoryPoint keeps historical curvature");

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

rsim_plugin::ActorState MakeEgoActor()
{
    rsim_plugin::ActorState actor{};
    actor.id = 1;
    actor.type = 0;
    actor.x = 0.0;
    actor.y = 0.0;
    actor.h = 0.0;
    actor.speed = 10.0;
    actor.acc_x = 1.2;
    actor.acc_y = 1.6;
    return actor;
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

std::vector<rsim_driver::PlanningTrajectoryPoint> MakeReusableTrajectory()
{
    std::vector<rsim_driver::PlanningTrajectoryPoint> trajectory;
    trajectory.reserve(81);
    for (int i = -40; i <= 40; ++i)
    {
        trajectory.push_back(MakePoint(1.0 + static_cast<double>(i) * 0.01,
                                       static_cast<double>(i) * 0.1,
                                       static_cast<double>(i) * 0.2,
                                       static_cast<double>(i) * 0.01,
                                       static_cast<double>(i) * 0.001,
                                       8.0 + static_cast<double>(i),
                                       static_cast<double>(i) * 0.1));
    }
    return trajectory;
}

}  // namespace

int main()
{
    rsim_driver::PlanningStartConfig config;
    config.planningPeriod = 0.1;
    config.mismatchDistanceThreshold = 0.3;

    const double currentTime = 1.0;
    const rsim_plugin::ActorState ego = MakeEgoActor();
    const double egoAccel =
        std::sqrt(ego.acc_x * ego.acc_x + ego.acc_y * ego.acc_y);
    rsim_driver::PlanningStart planningStart(config);

    rsim_driver::PlanningStartResult result =
        planningStart.Compute(ego, currentTime, {});
    const rsim_driver::PlanningStartPoint& start = result.start_point;
    if (!Require(start.source == rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "empty previous trajectory should use extrapolated point"))
        return 1;
    if (!Require(Near(start.x, 1.01, 1e-9) &&
                     Near(start.y, 0.0, 1e-9),
                 "kinematic extrapolation should use short straight-line motion"))
        return 1;
    if (!Require(Near(start.heading, ego.h, 1e-9) &&
                     Near(start.accel, egoAccel, 1e-9) &&
                     Near(result.start_curvature, 0.0, 1e-9) &&
                     Near(start.speed, 10.2, 1e-9),
                 "kinematic extrapolation should keep heading/accel and update speed"))
        return 1;
    if (!Require(Near(start.time, 1.1, 1e-9) &&
                     Near(result.start_curvature, 0.0, 1e-9) &&
                     result.stitching_trajectory.empty(),
                 "fallback extrapolation should have target time, zero curvature and empty stitching"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> tooShortForCurrent = {
        MakePoint(0.50, -5.0),
        MakePoint(0.90, -1.0),
    };
    result = planningStart.Compute(ego,
                                   currentTime,
                                   tooShortForCurrent);
    if (!Require(result.start_point.source ==
                     rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "trajectory that does not cover current time should fall back"))
        return 1;
    if (!Require(Near(result.start_curvature, 0.0, 1e-9) &&
                     result.stitching_trajectory.empty(),
                 "current-time fallback should not output curvature or stitching"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> mismatchedTrajectory = {
        MakePoint(1.00, 1.0),
        MakePoint(1.20, 2.0),
    };
    result = planningStart.Compute(ego,
                                   currentTime,
                                   mismatchedTrajectory);
    if (!Require(result.start_point.source ==
                     rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "tracking mismatch should use extrapolated point"))
        return 1;
    if (!Require(Near(result.start_point.matchDistance, 1.0, 1e-9) &&
                     Near(result.start_curvature, 0.0, 1e-9) &&
                     result.stitching_trajectory.empty(),
                 "tracking mismatch should report distance and no curvature or stitching"))
        return 1;

    std::vector<rsim_driver::PlanningTrajectoryPoint> tooShortForTarget = {
        MakePoint(1.000, 0.0),
        MakePoint(1.050, 0.5),
    };
    result = planningStart.Compute(ego,
                                   currentTime,
                                   tooShortForTarget);
    if (!Require(result.start_point.source ==
                     rsim_driver::PlanningStartSource::KinematicExtrapolation,
                 "trajectory that does not cover target time should fall back"))
        return 1;
    if (!Require(Near(result.start_point.matchDistance, 0.0, 1e-9) &&
                     Near(result.start_curvature, 0.0, 1e-9) &&
                     result.stitching_trajectory.empty(),
                 "target-time fallback should keep match distance and no curvature or stitching"))
        return 1;

    rsim_driver::PlanningStartConfig reusableConfig = config;
    reusableConfig.planningPeriod = 0.105;
    planningStart.SetConfig(reusableConfig);
    const std::vector<rsim_driver::PlanningTrajectoryPoint> reusableTrajectory =
        MakeReusableTrajectory();
    result = planningStart.Compute(ego,
                                   currentTime,
                                   reusableTrajectory);
    if (!Require(result.start_point.source ==
                     rsim_driver::PlanningStartSource::PreviousTrajectory,
                 "well-tracked trajectory should reuse previous trajectory"))
        return 1;
    if (!Require(Near(result.start_point.x, 1.05, 1e-9) &&
                     Near(result.start_point.y, 2.10, 1e-9) &&
                     Near(result.start_point.heading, 0.105, 1e-9) &&
                     Near(result.start_point.speed, 18.5, 1e-9) &&
                     Near(result.start_point.accel, 1.05, 1e-9) &&
                     Near(result.start_point.time, 1.105, 1e-9) &&
                     Near(result.start_curvature, 0.0105, 1e-9),
                 "previous trajectory start and curvature should use standard interpolation"))
        return 1;
    if (!Require(result.stitching_trajectory.size() == 30 &&
                     Near(result.stitching_trajectory.front().time, 0.81, 1e-9) &&
                     Near(result.stitching_trajectory.back().time, 1.10, 1e-9),
                 "previous trajectory reuse should output nearest 30 earlier points"))
        return 1;
    for (std::size_t i = 1; i < result.stitching_trajectory.size(); ++i)
    {
        if (!Require(result.stitching_trajectory[i - 1].time <
                         result.stitching_trajectory[i].time,
                     "stitching trajectory should be sorted from early to late"))
            return 1;
    }
    if (!Require(result.stitching_trajectory.back().time <
                     result.start_point.time,
                 "stitching trajectory should stay before planning start time"))
        return 1;

    // ---- PlanningStart::ToFrenet 测试 ----
    struct RefPoint
    {
        double x = 0.0;
        double y = 0.0;
        double hdg = 0.0;
        double k = 0.0;
        double dk = 0.0;
        double s = 0.0;
    };

    const std::vector<RefPoint> straightReference = {
        {-10.0, 0.0, 0.0, 0.0, 0.0, -10.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0, 0.0, 0.0, 10.0},
        {20.0, 0.0, 0.0, 0.0, 0.0, 20.0},
    };

    rsim_driver::CartesianFrenetState frenet;
    if (!Require(planningStart.ToFrenet(result, straightReference, &frenet),
                 "ToFrenet should succeed on straight reference line"))
        return 1;
    if (!Require(Near(frenet.s, 1.05, 1e-6) &&
                     Near(frenet.l, 2.10, 1e-6) &&
                     Near(frenet.s_dot, 18.4, 0.1),
                 "ToFrenet should produce correct s, l, s_dot on straight reference line"))
        return 1;

    rsim_driver::CartesianFrenetState emptyFrenet;
    if (!Require(!planningStart.ToFrenet(result,
                                         std::vector<RefPoint>{},
                                         &emptyFrenet),
                 "ToFrenet should fail on empty reference points"))
        return 1;

    std::fprintf(stderr, "PASS planning_start_point smoke\n");
    return 0;
}
