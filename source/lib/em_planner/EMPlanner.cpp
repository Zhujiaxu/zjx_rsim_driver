/*
 * EMPlanner — DP+QP staged local planner implementation.
 */
#include "EMPlanner.hpp"
#include "CollisionChecker.hpp"
#include "DpPathOptimizer.hpp"
#include "DpSpeedOptimizer.hpp"
#include "PathTimeTrajectory.hpp"
#include "PolynomialSolver.hpp"
#include "QpPathOptimizer.hpp"
#include "QpSpeedOptimizer.hpp"
#include "StGraph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

namespace rsim_driver
{
namespace
{
double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double Sqr(double v) { return v * v; }

bool LateralOverlap(double d0, double width0, double d1, double width1, double margin)
{
    return std::fabs(d0 - d1) <= (width0 + width1) * 0.5 + margin;
}
}  // namespace

// ============================================================================
//  IPlanner adapter
// ============================================================================

PlannerOutput EMPlanner::Plan(const PlannerInput& input)
{
    EMPlannerInput emIn;
    emIn.ego           = input.ego;
    emIn.desiredSpeed  = input.desiredSpeed;
    emIn.simTime       = input.simTime;
    emIn.planningDt    = input.planningDt;
    emIn.horizonTime   = input.horizonTime;
    emIn.referenceLine = input.referenceLine;
    emIn.obstacles     = input.obstacles;
    emIn.numObstacles  = input.numObstacles;
    // reference (legacy) left at defaults; DP+QP pipeline will use referenceLine

    EMPlannerOutput emOut = Plan(emIn);

    PlannerOutput out;
    out.valid      = emOut.valid;
    out.frenet     = emOut.frenet;
    out.trajectory = emOut.trajectory;
    return out;
}

// ============================================================================
//  Plan(EMPlannerInput) — dispatch
// ============================================================================

EMPlannerOutput EMPlanner::Plan(const EMPlannerInput& input) const
{
    // If a reference line is available, use the DP+QP pipeline.
    if (input.referenceLine != nullptr && !input.referenceLine->empty())
    {
        EMPlannerOutput output;
        if (PlanWithReferenceLine(input, &output))
        {
            output.valid = true;
            return output;
        }
        std::snprintf(output.debug.failureReason, sizeof(output.debug.failureReason),
                      "DP/QP pipeline failed, falling back to legacy");
        // Fall through to legacy
    }

    // --- Legacy path (backward compat) ---
    EMPlannerOutput output;
    output.pathDecision = SelectPath(input);
    if (output.pathDecision.cost >= LARGE_NUMBER * 0.5)
    {
        std::snprintf(output.debug.failureReason, sizeof(output.debug.failureReason),
                      "no feasible path");
        return output;
    }

    output.speedDecision = SelectSpeed(input, output.pathDecision);
    if (output.speedDecision.cost >= LARGE_NUMBER * 0.5)
    {
        std::snprintf(output.debug.failureReason, sizeof(output.debug.failureReason),
                      "no feasible speed");
        return output;
    }

    output.debug.bestPathCost  = output.pathDecision.cost;
    output.debug.bestSpeedCost = output.speedDecision.cost;
    output.targetSpeed = output.speedDecision.targetSpeed;
    output.targetD     = output.pathDecision.targetD;

    if (!BuildTrajectory(input, output.pathDecision, output.speedDecision, &output))
    {
        std::snprintf(output.debug.failureReason, sizeof(output.debug.failureReason),
                      "trajectory build failed");
        return output;
    }

    output.valid = true;
    return output;
}

PathDecision EMPlanner::SelectPath(const EMPlannerInput& input) const
{
    static constexpr int MAX_CANDIDATES = 8;
    PathDecision candidates[MAX_CANDIDATES];
    int count = 0;

    auto add = [&](PathDecisionType type, double targetD, double baseCost) {
        if (count >= MAX_CANDIDATES) return;
        targetD = Clamp(targetD, input.reference.rightBound, input.reference.leftBound);
        for (int i = 0; i < count; ++i)
            if (std::fabs(candidates[i].targetD - targetD) < 1e-3)
                return;
        candidates[count].type = type;
        candidates[count].targetD = targetD;
        candidates[count].cost = baseCost;
        ++count;
    };

    add(PathDecisionType::KEEP_LANE, 0.0, 0.0);
    add(PathDecisionType::KEEP_LANE, input.ego.d, 0.2);
    add(PathDecisionType::NUDGE_LEFT, input.ego.d + params.nudgeDistance, 1.5);
    add(PathDecisionType::NUDGE_RIGHT, input.ego.d - params.nudgeDistance, 1.5);
    if (input.reference.hasSameDirLeft)
        add(PathDecisionType::BORROW_LEFT, input.reference.laneWidth, 4.0);
    if (input.reference.hasSameDirRight)
        add(PathDecisionType::BORROW_RIGHT, -input.reference.laneWidth, 4.0);

    PathDecision best;
    for (int i = 0; i < count; ++i)
    {
        PathDecision candidate = candidates[i];
        candidate.cost += 0.4 * std::fabs(candidate.targetD);
        candidate.cost += 0.15 * std::fabs(candidate.targetD - input.ego.d);

        if (input.reference.distToJunction < params.junctionReturnDist)
            candidate.cost += 3.0 * std::fabs(candidate.targetD);

        bool blocked = false;
        for (int oi = 0; input.obstacles != nullptr && oi < input.numObstacles; ++oi)
        {
            const Obstacle& obs = input.obstacles[oi];
            double ds = obs.s - input.ego.s;
            if (ds < -params.egoLength || ds > 45.0) continue;
            if (!LateralOverlap(candidate.targetD, params.egoWidth,
                                obs.d, obs.width, params.safeMarginLat))
                continue;

            bool canStop = ds > params.safeGapLong + obs.length * 0.5;
            if (!canStop && std::fabs(candidate.targetD) < input.reference.laneWidth * 0.5)
                blocked = true;
            candidate.cost += canStop ? 20.0 / std::max(1.0, ds) : 100.0;
        }

        if (blocked) continue;
        if (candidate.cost < best.cost) best = candidate;
    }

    if (best.cost >= LARGE_NUMBER * 0.5)
    {
        best.type = PathDecisionType::STOP_ONLY;
        best.targetD = Clamp(input.ego.d, input.reference.rightBound, input.reference.leftBound);
        best.cost = 1000.0;
    }
    return best;
}

SpeedDecision EMPlanner::SelectSpeed(const EMPlannerInput& input, const PathDecision& path) const
{
    SpeedDecision best;
    best.type = SpeedDecisionType::CRUISE;
    best.targetSpeed = Clamp(input.desiredSpeed, 0.0, params.maxSpeed);
    best.stopS = LARGE_NUMBER;
    best.cost = std::fabs(best.targetSpeed - input.ego.s_d);

    double nearestBlockingS = LARGE_NUMBER;
    double nearestBlockingSpeed = best.targetSpeed;
    for (int i = 0; input.obstacles != nullptr && i < input.numObstacles; ++i)
    {
        const Obstacle& obs = input.obstacles[i];
        if (obs.s <= input.ego.s) continue;
        if (!LateralOverlap(path.targetD, params.egoWidth, obs.d, obs.width, params.safeMarginLat))
            continue;
        if (obs.s < nearestBlockingS)
        {
            nearestBlockingS = obs.s;
            nearestBlockingSpeed = std::max(0.0, obs.speed);
        }
    }

    if (nearestBlockingS < LARGE_NUMBER * 0.5)
    {
        double gap = nearestBlockingS - input.ego.s;
        double stopS = nearestBlockingS - params.safeGapLong - params.stopMargin;
        if (nearestBlockingSpeed < 0.5)
        {
            best.type = SpeedDecisionType::STOP;
            best.targetSpeed = 0.0;
            best.stopS = std::max(input.ego.s, stopS);
            best.cost = 20.0 + std::max(0.0, params.safeGapLong - gap);
        }
        else if (gap < params.safeGapLong + params.egoLength)
        {
            best.type = SpeedDecisionType::STOP;
            best.targetSpeed = 0.0;
            best.stopS = std::max(input.ego.s, stopS);
            best.cost = 100.0 - std::min(90.0, gap);
        }
        else
        {
            best.type = nearestBlockingSpeed < best.targetSpeed
                            ? SpeedDecisionType::FOLLOW
                            : SpeedDecisionType::YIELD;
            double brakingLimit =
                std::sqrt(std::max(0.0, 2.0 * params.maxDecel * std::max(0.0, gap - params.safeGapLong)));
            best.targetSpeed = Clamp(std::min(nearestBlockingSpeed, brakingLimit), 0.0, best.targetSpeed);
            best.stopS = stopS;
            best.cost = 10.0 + std::fabs(best.targetSpeed - input.ego.s_d);
        }
    }

    if (path.type == PathDecisionType::STOP_ONLY)
    {
        best.type = SpeedDecisionType::STOP;
        best.targetSpeed = 0.0;
        best.stopS = input.ego.s + std::max(1.0, Sqr(input.ego.s_d) / (2.0 * params.maxDecel));
        best.cost += 100.0;
    }
    return best;
}

bool EMPlanner::BuildTrajectory(const EMPlannerInput& input,
                                const PathDecision& path,
                                const SpeedDecision& speed,
                                EMPlannerOutput* output) const
{
    if (output == nullptr) return false;

    double T = Clamp(input.horizonTime, 1.0, 5.0);
    if (speed.type == SpeedDecisionType::STOP && speed.stopS < LARGE_NUMBER * 0.5)
    {
        double stopDistance = std::max(0.5, speed.stopS - input.ego.s);
        double avgSpeed = std::max(0.5, input.ego.s_d * 0.5);
        T = Clamp(stopDistance / avgSpeed, 1.0, 5.0);
    }
    FrenetTrajectory frenet;
    if (!BuildLongitudinalQuartic(input.ego.s, std::max(0.0, input.ego.s_d), input.ego.s_dd,
                                  speed.targetSpeed, 0.0, T, frenet.sCoeffs))
        return false;
    if (!BuildLateralQuintic(input.ego.d, input.ego.d_d, input.ego.d_dd,
                             path.targetD, 0.0, 0.0, T, frenet.dCoeffs))
        return false;

    frenet.T = T;
    frenet.valid = true;
    frenet.cost = path.cost + speed.cost;

    PlannedTrajectory trajectory;
    double dt = std::max(0.05, params.trajectoryTimeStep);
    int count = 0;
    double prevSpeed = input.ego.s_d;
    double prevDdot = input.ego.d_d;
    for (double t = 0.0; t <= T + 1e-6 && count < PlannedTrajectory::MAX_POINTS; t += dt)
    {
        TrajectoryPoint& p = trajectory.points[count];
        p.t = t;
        p.s = frenet.EvalS(t);
        p.d = Clamp(frenet.EvalD(t), input.reference.rightBound, input.reference.leftBound);
        p.speed = std::max(0.0, frenet.EvalSdot(t));
        p.heading = std::atan2(frenet.EvalDdot(t), std::max(0.1, p.speed));

        if (count > 0)
        {
            double accel = (p.speed - prevSpeed) / dt;
            double lateralAccel = (frenet.EvalDdot(t) - prevDdot) / dt;
            if (accel > params.maxAccel + 1e-6 || accel < -params.maxDecel - 1e-6)
                return false;
            if (std::fabs(lateralAccel) > params.maxLateralAccel + 1e-6)
                return false;
        }
        prevSpeed = p.speed;
        prevDdot = frenet.EvalDdot(t);
        ++count;
    }

    trajectory.numPoints = count;
    trajectory.startTime = input.simTime;
    if (trajectory.numPoints < 2) return false;
    if (HasCollision(trajectory, input.obstacles, input.numObstacles,
                     params.egoLength, params.egoWidth,
                     params.collisionLongBuffer, params.safeMarginLat))
        return false;

    output->frenet = frenet;
    output->trajectory = trajectory;
    return true;
}

bool EMPlanner::PlanWithReferenceLine(const EMPlannerInput& input,
                                      EMPlannerOutput* output) const
{
    if (output == nullptr || input.referenceLine == nullptr ||
        input.referenceLine->empty())
    {
        return false;
    }

    EMPlannerInput legacyInput = input;
    legacyInput.reference.leftBound = 2.0;
    legacyInput.reference.rightBound = -2.0;
    legacyInput.reference.laneWidth = 3.5;

    EMPlannerOutput legacyOutput;
    legacyOutput.pathDecision = SelectPath(legacyInput);
    if (legacyOutput.pathDecision.cost < LARGE_NUMBER * 0.5)
    {
        legacyOutput.speedDecision = SelectSpeed(legacyInput, legacyOutput.pathDecision);
        if (legacyOutput.speedDecision.cost < LARGE_NUMBER * 0.5 &&
            BuildTrajectory(legacyInput, legacyOutput.pathDecision,
                            legacyOutput.speedDecision, &legacyOutput))
        {
            legacyOutput.debug.bestPathCost = legacyOutput.pathDecision.cost;
            legacyOutput.debug.bestSpeedCost = legacyOutput.speedDecision.cost;
            legacyOutput.targetSpeed = legacyOutput.speedDecision.targetSpeed;
            legacyOutput.targetD = legacyOutput.pathDecision.targetD;
            *output = legacyOutput;
            return true;
        }
    }

    ConstantVelocityPredictor defaultPredictor;
    const IPredictor& predictor =
        input.predictor != nullptr ? *input.predictor : defaultPredictor;

    std::vector<DpPathPoint> dpPath;
    DpPathOptimizer dpPathOptimizer;
    if (!dpPathOptimizer.Optimize(*input.referenceLine, input.ego,
                                  input.obstacles, input.numObstacles,
                                  predictor, &dpPath))
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "DP path optimization failed");
        return false;
    }

    std::vector<double> sGrid;
    sGrid.reserve(dpPath.size());
    for (const DpPathPoint& p : dpPath)
        sGrid.push_back(p.s);

    std::vector<double> lb;
    std::vector<double> ub;
    BuildLateralBounds(*input.referenceLine, input.ego.s,
                       input.obstacles, input.numObstacles,
                       predictor, sGrid, &lb, &ub);
    if (lb.size() != dpPath.size() || ub.size() != dpPath.size())
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "lateral bound build failed");
        return false;
    }

    std::vector<QpPathPoint> qpPath;
    QpPathOptimizer qpPathOptimizer;
    if (!qpPathOptimizer.Optimize(*input.referenceLine, dpPath, lb, ub, &qpPath))
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "QP path optimization failed");
        return false;
    }

    std::vector<SpeedPathPoint> speedPath;
    speedPath.reserve(qpPath.size());
    for (const QpPathPoint& p : qpPath)
    {
        SpeedPathPoint sp;
        sp.s = p.s;
        double heading = 0.0;
        input.referenceLine->Eval(p.s, p.l, &sp.x, &sp.y, &heading);
        sp.s = p.s;
        speedPath.push_back(sp);
    }

    constexpr double stDt = 0.5;
    const std::vector<StBoundary> stBoundaries = StGraph::Build(
        speedPath, input.obstacles, input.numObstacles,
        input.horizonTime, stDt, params.egoLength, params.egoLength);

    double speedLimit = params.maxSpeed;
    for (const ReferencePoint& point : input.referenceLine->points)
    {
        if (point.s < input.ego.s)
            continue;
        if (point.s > input.ego.s + std::max(1.0, input.horizonTime) * speedLimit)
            break;
        const double k = std::fabs(point.k);
        if (k > 1e-4)
            speedLimit = std::min(speedLimit, std::sqrt(params.maxLateralAccel / k));
    }
    speedLimit = std::max(1.0, std::min(speedLimit, params.maxSpeed));

    std::vector<DpSpeedPoint> dpSpeed;
    DpSpeedOptimizer dpSpeedOptimizer;
    if (!dpSpeedOptimizer.Optimize(input.ego, input.desiredSpeed, speedLimit,
                                   stBoundaries, input.horizonTime, &dpSpeed))
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "DP speed optimization failed");
        return false;
    }

    std::vector<QpSpeedPoint> qpSpeed;
    QpSpeedOptimizer qpSpeedOptimizer;
    if (!qpSpeedOptimizer.Optimize(input.ego, dpSpeed, stBoundaries,
                                   speedLimit, params.maxAccel, params.maxDecel,
                                   &qpSpeed))
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "QP speed optimization failed");
        return false;
    }

    if (!PathTimeTrajectory::Combine(qpPath, qpSpeed, input.simTime,
                                     &output->frenet, &output->trajectory))
    {
        std::snprintf(output->debug.failureReason, sizeof(output->debug.failureReason),
                      "path-time trajectory combine failed");
        return false;
    }

    output->pathDecision.type = PathDecisionType::KEEP_LANE;
    output->pathDecision.targetD = qpPath.empty() ? input.ego.d : qpPath.back().l;
    output->pathDecision.cost = 0.0;
    output->speedDecision.type = SpeedDecisionType::CRUISE;
    output->speedDecision.targetSpeed = qpSpeed.empty()
                                            ? input.desiredSpeed
                                            : std::max(0.0, qpSpeed.back().v);
    output->speedDecision.cost = 0.0;
    output->targetD = output->pathDecision.targetD;
    output->targetSpeed = output->speedDecision.targetSpeed;
    output->debug.dpPathCost = 0.0;
    output->debug.qpPathCost = 0.0;
    output->debug.dpSpeedCost = 0.0;
    output->debug.qpSpeedCost = 0.0;
    return true;
}

void EMPlanner::BuildLateralBounds(const ReferenceLine& /*refLine*/,
                                   double /*egoS*/,
                                   const Obstacle* obstacles,
                                   int numObstacles,
                                   const IPredictor& /*predictor*/,
                                   const std::vector<double>& sGrid,
                                   std::vector<double>* lb,
                                   std::vector<double>* ub) const
{
    if (lb == nullptr || ub == nullptr)
        return;

    lb->assign(sGrid.size(), -2.0);
    ub->assign(sGrid.size(), 2.0);

    if (obstacles == nullptr || numObstacles <= 0)
        return;

    for (size_t i = 0; i < sGrid.size(); ++i)
    {
        for (int oi = 0; oi < numObstacles; ++oi)
        {
            const Obstacle& obs = obstacles[oi];
            if (std::fabs(obs.s - sGrid[i]) > params.safeGapLong)
                continue;

            if (obs.d >= 0.0)
                (*ub)[i] = std::min((*ub)[i], obs.d - 0.5 * obs.width - params.safeMarginLat);
            else
                (*lb)[i] = std::max((*lb)[i], obs.d + 0.5 * obs.width + params.safeMarginLat);
        }

        if ((*lb)[i] > (*ub)[i])
        {
            const double mid = 0.5 * ((*lb)[i] + (*ub)[i]);
            (*lb)[i] = mid - 0.05;
            (*ub)[i] = mid + 0.05;
        }
    }
}

}  // namespace rsim_driver
