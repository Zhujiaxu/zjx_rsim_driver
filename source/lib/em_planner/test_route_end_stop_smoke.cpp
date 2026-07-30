#include "EmPlanner.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-6)
{
    return std::fabs(actual - expected) <= tolerance;
}

struct TestReferencePoint
{
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
    double k = 0.0;
    double dk = 0.0;
    double s = 0.0;
};

std::vector<TestReferencePoint> MakeStraightReference(double endS)
{
    std::vector<TestReferencePoint> points;
    for (double s = 0.0; s < endS; s += 0.3)
        points.push_back({s, 0.0, 0.0, 0.0, 0.0, s});
    points.push_back({endS, 0.0, 0.0, 0.0, 0.0, endS});
    return points;
}

rsim_plugin::ActorState MakeStoppedEgo()
{
    rsim_plugin::ActorState ego{};
    ego.id = 1;
    ego.length = 4.0;
    ego.width = 2.0;
    return ego;
}

} // namespace

int main()
{
    constexpr double kStartS = 0.309086;
    constexpr double kReferenceEndS = 28.077513;

    const rsim_plugin::ActorState stoppedEgo = MakeStoppedEgo();
    const std::vector<rsim_plugin::ActorState> actors = {stoppedEgo};

    rsim_driver::EmPlanner integratedPlanner;
    rsim_driver::EmPlannerResult integratedResult;
    if (!Require(integratedPlanner.EMPlanPathDetailed(
                     actors, stoppedEgo.id, stoppedEgo, 0.0, {},
                     MakeStraightReference(20.0), &integratedResult) &&
                     integratedResult.stop_at_reference_end &&
                     Near(integratedResult.dp_result.path.back().s, 20.0) &&
                     Near(integratedResult.qp_result.localfrenetpath.back().s,
                          20.0),
                 "integrated path planning should detect the route end") ||
        !Require(integratedPlanner.EMPlanSpeedDetailed(
                     actors, stoppedEgo.id, &integratedResult) &&
                     integratedResult.speed_qp_increase_points_success &&
                     Near(integratedResult.speed_qp_result.stpoints.back().v,
                          0.0, 2e-5) &&
                     Near(integratedResult.speed_qp_result.stpoints.back().a,
                          0.0, 2e-5),
                 "integrated speed planning should produce a stationary end") ||
        !Require(integratedPlanner.EMPlanPostProcessDetailed(integratedResult) &&
                     !integratedResult.trajectory.empty() &&
                     Near(integratedResult.trajectory.back().x, 17.5,
                          2e-3) &&
                     Near(integratedResult.trajectory.back().speed, 0.0,
                          2e-5) &&
                     Near(integratedResult.trajectory.back().accel, 0.0,
                          2e-5),
                 "integrated post-processing should preserve the stop"))
        return 1;

    for (int shorterStage = 0; shorterStage < 2; ++shorterStage)
    {
        rsim_driver::EmPlannerConfig config;
        config.dp_config.s_step = 1.0;
        config.dp_config.s_step_count = shorterStage == 0 ? 20 : 10;
        config.qp_config.ds = 1.0;
        config.qp_config.num_points = shorterStage == 0 ? 11 : 21;
        rsim_driver::EmPlanner planner(config);
        rsim_driver::EmPlannerResult result;
        if (!Require(planner.EMPlanPathDetailed(
                         actors, stoppedEgo.id, stoppedEgo, 0.0, {},
                         MakeStraightReference(15.0), &result) &&
                         !result.stop_at_reference_end &&
                         Near(result.dp_result.path.back().s, 10.0) &&
                         Near(result.qp_result.localfrenetpath.back().s, 10.0),
                     "path planning should use the shorter shared horizon"))
            return 1;
    }

    rsim_driver::StartPointFrenetState pathStart;
    pathStart.s = kStartS;

    rsim_driver::DpPlanner pathDp;
    rsim_driver::DpPlannerResult dpResult;
    if (!Require(pathDp.Plan(pathStart, {}, &dpResult, kReferenceEndS) &&
                     Near(dpResult.path.back().s, kReferenceEndS),
                 "path DP should end exactly at the reference boundary"))
        return 1;

    rsim_driver::DPIncreasePoints dpIncrease;
    rsim_driver::DpIncreasePointsResult denseDp;
    if (!Require(dpIncrease.increasepoints(dpResult, &denseDp),
                 "limited DP path should densify"))
        return 1;

    rsim_driver::DrivableAreaBuilder areaBuilder;
    rsim_driver::DrivableAreaResult area;
    if (!Require(areaBuilder.Build(denseDp.path, {}, &area),
                 "limited DP path should build a drivable area"))
        return 1;

    rsim_driver::QpPathOptimizer pathQp;
    rsim_driver::QpPathResult qpResult;
    if (!Require(pathQp.Optimize(pathStart, area, &qpResult,
                                kReferenceEndS) &&
                     Near(qpResult.localfrenetpath.back().s,
                          kReferenceEndS),
                 "path QP should end exactly at the reference boundary"))
        return 1;

    rsim_driver::QpIncreasePoints qpIncrease;
    rsim_driver::QpIncreasePointsResult denseQp;
    if (!Require(qpIncrease.increasepoints(qpResult, &denseQp),
                 "limited QP path should densify"))
        return 1;

    const auto reference = MakeStraightReference(kReferenceEndS);
    std::vector<rsim_driver::CartesianPathPoint> cartesian;
    if (!Require(rsim_driver::FrenetPathToCartesian(
                     reference, denseQp.localfrenetpath, &cartesian),
                 "limited Frenet path should convert to Cartesian"))
        return 1;

    std::vector<rsim_driver::SpeedReferenceLinePoint> speedReference;
    if (!Require(rsim_driver::SpeedReferencePathGenerator(
                     cartesian, &speedReference),
                 "limited Cartesian path should produce a speed reference"))
        return 1;

    rsim_driver::DpPathPoint outOfRange;
    outOfRange.s = kReferenceEndS + 0.1;
    rsim_driver::CartesianPathPoint invalidCartesian;
    if (!Require(!rsim_driver::FrenetPointToCartesian(
                     reference, outOfRange, &invalidCartesian),
                 "true Frenet s overflow should fail"))
        return 1;

    rsim_driver::DpPathPoint boundaryNoise;
    boundaryNoise.s = -0.032072502;
    if (!Require(rsim_driver::FrenetPointToCartesian(
                     reference, boundaryNoise, &invalidCartesian),
                 "small smoothing error at the reference start should normalize"))
        return 1;

    rsim_driver::DynamicPlanSpeedPoint speedStart;
    speedStart.v = 6.0;
    rsim_driver::DynamicPlanSpeedPlanner speedDp;
    rsim_driver::DynamicPlanSpeedResult speedDpResult;
    if (!Require(speedDp.Plan(speedStart, {}, &speedDpResult, 28.0, true) &&
                     Near(speedDpResult.stpoints.back().t, 8.0) &&
                     Near(speedDpResult.stpoints.back().s, 28.0),
                 "route-end speed DP should use the full time and space horizon"))
        return 1;

    rsim_driver::StDrivableAreaBuilder stAreaBuilder;
    rsim_driver::StDrivableAreaResult stArea;
    if (!Require(stAreaBuilder.Build({}, &speedDpResult, &stArea),
                 "route-end speed DP should build an ST drivable area"))
        return 1;

    rsim_driver::SpeedQpOptimizer speedQp;
    rsim_driver::QpSpeedOptimizerResult speedQpResult;
    if (!Require(speedQp.Optimize(speedStart, stArea, &speedQpResult, true),
                 "route-end speed QP should solve"))
        return 1;
    if (!Require(Near(speedQpResult.stpoints.back().v, 0.0, 2e-5),
                 "route-end speed QP should enforce terminal zero speed"))
        return 1;
    if (!Require(Near(speedQpResult.stpoints.back().a, 0.0, 2e-5),
                 "route-end speed QP should enforce terminal zero acceleration"))
        return 1;
    for (std::size_t i = 0; i < speedQpResult.stpoints.size(); ++i)
    {
        const auto &point = speedQpResult.stpoints[i];
        if (!Require(point.v >= -2e-5,
                     "route-end speed QP should not reverse") ||
            !Require(i == 0 ||
                         point.s + 2e-5 >= speedQpResult.stpoints[i - 1].s,
                     "route-end speed QP progress should be monotonic"))
            return 1;
    }
    if (!Require(Near(speedQpResult.stpoints.back().s, 25.5, 2e-3),
                 "ego center should stop at the safety-adjusted endpoint"))
        return 1;

    rsim_driver::QpSpeedIncreasePoints speedIncrease;
    std::vector<rsim_driver::DynamicPlanSpeedPoint> denseSpeed;
    if (!Require(speedIncrease.increasepoints(speedQpResult, &denseSpeed),
                 "route-end speed QP result should densify"))
        return 1;

    rsim_driver::DynamicPlanSpeedPoint lowSpeedStart;
    lowSpeedStart.v = 1.858962140;
    lowSpeedStart.a = -0.594565589;
    rsim_driver::DynamicPlanSpeedResult lowSpeedDpResult;
    rsim_driver::StDrivableAreaResult lowSpeedArea;
    rsim_driver::QpSpeedOptimizerResult lowSpeedQpResult;
    std::vector<rsim_driver::DynamicPlanSpeedPoint> lowSpeedDense;
    const bool lowDpOk = speedDp.Plan(lowSpeedStart, {}, &lowSpeedDpResult,
                                      8.15, true);
    const bool lowAreaOk = lowDpOk &&
                           stAreaBuilder.Build({}, &lowSpeedDpResult,
                                               &lowSpeedArea);
    const bool lowQpOk = lowAreaOk &&
                         speedQp.Optimize(lowSpeedStart, lowSpeedArea,
                                          &lowSpeedQpResult, true);
    if (!Require(lowDpOk && lowAreaOk && lowQpOk,
                 "low-speed route-end planning should solve"))
        return 1;
    if (!Require(Near(lowSpeedQpResult.stpoints.back().v, 0.0, 2e-5) &&
                     Near(lowSpeedQpResult.stpoints.back().a, 0.0, 2e-5) &&
                     Near(lowSpeedQpResult.stpoints.back().s, 5.65, 2e-3),
                 "low-speed route-end result should finish stationary") ||
        !Require(speedIncrease.increasepoints(lowSpeedQpResult,
                                              &lowSpeedDense),
                 "low-speed route-end result should densify"))
        return 1;

    std::fprintf(stderr, "PASS route_end_stop smoke\n");
    return 0;
}
