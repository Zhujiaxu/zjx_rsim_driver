/*
 * SamplingPlanner — implementation.
 */

#include "SamplingPlanner.hpp"
#include "CollisionChecker.hpp"
#include "PolynomialSolver.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{
namespace
{
double Clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

double Square(double v)
{
    return v * v;
}

double EvalPoly4DDDot(const double c[5], double t)
{
    return 6.0 * c[3] + t * 24.0 * c[4];
}

double EvalPoly5DDDot(const double c[6], double t)
{
    return 6.0 * c[3] + t * (24.0 * c[4] + t * 60.0 * c[5]);
}
}  // namespace

PlannerOutput SamplingPlanner::Plan(const PlannerInput& input)
{
    const double roadLeftBound  = 3.5;   // default half-lane
    const double roadRightBound = -3.5;

    TrajectoryPlanResult result = PlanTrajectory(
        input.ego, input.desiredSpeed,
        LARGE_NUMBER,  // stopS — no stop target from the generic interface
        input.obstacles, input.numObstacles,
        roadLeftBound, roadRightBound,
        LARGE_NUMBER); // distToJunction — unknown

    PlannerOutput out;
    out.valid      = result.valid;
    out.frenet     = result.frenet;
    out.trajectory = result.trajectory;
    return out;
}

bool SamplingPlanner::IsPathBlocked(double dTarget, double currentRefS,
                                    const Obstacle* obstacles, int numObs)
{
    for (int i = 0; i < numObs; ++i)
    {
        double longDist = obstacles[i].s - currentRefS;
        if (longDist <= 0 || longDist > params.safeGapLong)
            continue;

        double latDist   = std::fabs(obstacles[i].d - dTarget);
        double threshold = (params.egoWidth + obstacles[i].width) / 2.0 + params.safeMarginLat;
        if (latDist < threshold)
            return true;
    }
    return false;
}

double SamplingPlanner::ComputeSafeSpeed(double dTarget, double currentRefS,
                                         double desiredSpeed, double stopS,
                                         const Obstacle* obstacles, int numObs)
{
    double minGap = LARGE_NUMBER;
    double leadSpeed = 0.0;
    for (int i = 0; i < numObs; ++i)
    {
        double longDist = obstacles[i].s - currentRefS;
        if (longDist <= 0)
            continue;

        double latDist   = std::fabs(obstacles[i].d - dTarget);
        double threshold = (params.egoWidth + obstacles[i].width) / 2.0 + params.safeMarginLat;
        if (latDist < threshold)
        {
            double gap = longDist - obstacles[i].length / 2.0 - params.egoLength / 2.0;
            gap = std::max(0.0, gap);
            if (gap < minGap)
            {
                minGap = gap;
                leadSpeed = std::max(0.0, obstacles[i].speed);
            }
        }
    }

    double safeSpeed = desiredSpeed;

    if (minGap < LARGE_NUMBER)
    {
        double brakingDist = desiredSpeed * desiredSpeed / (2.0 * params.comfortDecel);
        if (minGap < brakingDist)
        {
            safeSpeed = std::sqrt(2.0 * params.comfortDecel * minGap);
            if (leadSpeed > 0.1 && minGap > params.collisionLongBuffer)
            {
                safeSpeed = std::max(safeSpeed, leadSpeed);
            }
        }
    }

    double distToStop = stopS - currentRefS;
    if (distToStop > 0)
    {
        double brakingDist = desiredSpeed * desiredSpeed / (2.0 * params.comfortDecel);
        double earlyMargin = desiredSpeed * 3.0;
        if (distToStop < brakingDist + earlyMargin)
        {
            double margin    = 1.0;
            double stopSpeed = std::sqrt(2.0 * params.comfortDecel * std::max(0.0, distToStop - margin));
            safeSpeed        = std::min(safeSpeed, stopSpeed);
        }
    }
    if (distToStop < 0.5)
    {
        safeSpeed = 0.0;
    }

    return std::max(0.0, safeSpeed);
}

bool SamplingPlanner::BuildPlannedTrajectory(const FrenetTrajectory& frenet,
                                             double roadLeftBound,
                                             double roadRightBound,
                                             PlannedTrajectory* out) const
{
    if (!out || !frenet.valid || frenet.T <= 0.0)
        return false;

    *out = PlannedTrajectory {};
    const double dt = std::max(0.05, params.sampleTimeStep);
    int count = 0;
    for (double t = 0.0; t <= frenet.T + 1e-6 && count < PlannedTrajectory::MAX_POINTS; t += dt)
    {
        TrajectoryPoint& p = out->points[count];
        p.t       = t;
        p.s       = frenet.EvalS(t);
        p.d       = frenet.EvalD(t);
        p.speed   = std::max(0.0, frenet.EvalSdot(t));
        p.heading = std::atan2(frenet.EvalDdot(t), std::max(0.1, p.speed));

        const double sdd = frenet.EvalSddot(t);
        const double dd  = frenet.EvalDdot(t);
        const double ddd = frenet.EvalDddot(t);
        const double denom = std::pow(std::max(0.1, p.speed * p.speed + dd * dd), 1.5);
        p.curvature = denom > 1e-6 ? std::fabs(p.speed * ddd - dd * sdd) / denom : 0.0;

        if (p.d < roadRightBound || p.d > roadLeftBound)
            return false;
        if (p.speed > params.maxSpeed + 1e-3)
            return false;

        ++count;
    }
    out->numPoints = count;
    return count >= 2;
}

// HasCollision / MinObstacleClearance are now shared free functions in CollisionChecker.hpp.

bool SamplingPlanner::EvaluateTrajectory(const FrenetTrajectory& frenet,
                                         const FrenetState& current,
                                         double desiredSpeed,
                                         double stopS,
                                         const Obstacle* obstacles,
                                         int numObs,
                                         double roadLeftBound,
                                         double roadRightBound,
                                         double distToJunction,
                                         double* cost,
                                         PlannedTrajectory* out) const
{
    PlannedTrajectory trajectory;
    if (!BuildPlannedTrajectory(frenet, roadLeftBound, roadRightBound, &trajectory))
        return false;

    double smoothCost = 0.0;
    double accelCost  = 0.0;
    const double dt = std::max(0.05, params.sampleTimeStep);
    for (double t = 0.0; t <= frenet.T + 1e-6; t += dt)
    {
        double sDot   = frenet.EvalSdot(t);
        double sDDot  = frenet.EvalSddot(t);
        double sJerk  = EvalPoly4DDDot(frenet.sCoeffs, t);
        double dDot   = frenet.EvalDdot(t);
        double dDDot  = frenet.EvalDddot(t);
        double dJerk  = EvalPoly5DDDot(frenet.dCoeffs, t);

        if (sDot < -1e-3 || sDot > params.maxSpeed)
            return false;
        if (sDDot > params.maxAccel || sDDot < -params.maxDecel)
            return false;
        if (std::fabs(sJerk) > params.maxJerk || std::fabs(dJerk) > params.maxJerk)
            return false;
        if (std::fabs(dDDot) > params.maxLateralAccel)
            return false;

        double s = frenet.EvalS(t);
        if (stopS < LARGE_NUMBER * 0.5 && s > stopS + params.stopMargin)
            return false;

        smoothCost += (sJerk * sJerk + dJerk * dJerk) * dt;
        accelCost  += (sDDot * sDDot + dDDot * dDDot) * dt;
    }

    if (HasCollision(trajectory, obstacles, numObs,
                     params.egoLength, params.egoWidth,
                     params.collisionLongBuffer, params.safeMarginLat))
        return false;

    const TrajectoryPoint& end = trajectory.points[trajectory.numPoints - 1];
    double terminalSpeedCost = Square(end.speed - desiredSpeed);
    double routeCost         = Square(end.d);
    double lateralCost       = Square(end.d - current.d);
    double timeCost          = frenet.T;
    double clearance         = MinObstacleClearance(trajectory, obstacles, numObs,
                                                      params.egoLength, params.egoWidth);
    double obstacleCost      = (clearance < LARGE_NUMBER * 0.5)
                             ? 1.0 / Square(std::max(0.5, clearance))
                             : 0.0;

    double total = params.wRoute * routeCost
                 + params.wSpeed * terminalSpeedCost
                 + params.wSmooth * (smoothCost + 0.2 * accelCost)
                 + params.wTime * timeCost
                 + params.wLateralChange * lateralCost
                 + params.wObstacleClearance * obstacleCost;

    if (distToJunction < params.junctionReturnDist && std::fabs(end.d) > 0.1)
    {
        double urgency = (params.junctionReturnDist - distToJunction) / params.junctionReturnDist;
        total += params.wJunction * urgency * end.d * end.d;
    }

    if (cost)
        *cost = total;
    if (out)
        *out = trajectory;
    return true;
}

CandidateResult SamplingPlanner::Plan(double           currentD,
                                      double           desiredSpeed,
                                      double           stopS,
                                      double           currentRefS,
                                      const Obstacle*  obstacles,
                                      int              numObs,
                                      double           roadLeftBound,
                                      double           roadRightBound,
                                      double           distToJunction)
{
    (void) currentD;  // reserved for future per-candidate transition cost

    static constexpr int MAX_CANDIDATES = 20;
    double               targets[MAX_CANDIDATES];
    int                  count = 0;

    targets[count++] = 0.0;
    for (double offset = params.lateralStep;
         offset <= params.lateralRange + 0.01 && count < MAX_CANDIDATES - 1;
         offset += params.lateralStep)
    {
        double dLeft  = offset;
        double dRight = -offset;
        if (dLeft <= roadLeftBound)
            targets[count++] = dLeft;
        if (dRight >= roadRightBound)
            targets[count++] = dRight;
    }

    CandidateResult best;
    for (int i = 0; i < count; ++i)
    {
        double d = targets[i];
        if (d < roadRightBound || d > roadLeftBound)
            continue;

        double safeSpeed = ComputeSafeSpeed(d, currentRefS, desiredSpeed, stopS, obstacles, numObs);
        bool   blocked   = IsPathBlocked(d, currentRefS, obstacles, numObs);

        double cost = params.wRoute * d * d
                    + params.wSpeed * (safeSpeed - desiredSpeed) * (safeSpeed - desiredSpeed);

        if (distToJunction < params.junctionReturnDist && std::fabs(d) > 0.1)
        {
            double urgency = (params.junctionReturnDist - distToJunction) / params.junctionReturnDist;
            cost += params.wJunction * urgency * d * d;
        }

        if (blocked)
            cost += 1000.0;

        if (cost < best.cost)
        {
            best.dTarget   = d;
            best.safeSpeed = safeSpeed;
            best.cost      = cost;
            best.feasible  = !blocked;
        }
    }

    return best;
}

TrajectoryPlanResult SamplingPlanner::PlanTrajectory(const FrenetState& current,
                                                     double desiredSpeed,
                                                     double stopS,
                                                     const Obstacle* obstacles,
                                                     int numObs,
                                                     double roadLeftBound,
                                                     double roadRightBound,
                                                     double distToJunction)
{
    TrajectoryPlanResult best;

    const double clampedDesired = Clamp(desiredSpeed, 0.0, params.maxSpeed);
    const double safeKeepSpeed = ComputeSafeSpeed(current.d, current.s, clampedDesired, stopS, obstacles, numObs);

    static constexpr int MAX_LATERAL_TARGETS = 16;
    double lateralTargets[MAX_LATERAL_TARGETS] = {};
    int lateralCount = 0;
    auto addTarget = [&](double d) {
        if (d < roadRightBound || d > roadLeftBound || lateralCount >= MAX_LATERAL_TARGETS)
            return;
        for (int i = 0; i < lateralCount; ++i)
        {
            if (std::fabs(lateralTargets[i] - d) < 1e-3)
                return;
        }
        lateralTargets[lateralCount++] = d;
    };

    addTarget(Clamp(0.0, roadRightBound, roadLeftBound));
    addTarget(Clamp(current.d, roadRightBound, roadLeftBound));
    for (double offset = params.lateralStep;
         offset <= params.lateralRange + 0.01;
         offset += params.lateralStep)
    {
        addTarget(offset);
        addTarget(-offset);
    }

    const double timeStep = std::max(0.5, params.trajectoryTimeStep);
    for (double T = params.minTrajectoryTime; T <= params.maxTrajectoryTime + 1e-6; T += timeStep)
    {
        for (int li = 0; li < lateralCount; ++li)
        {
            double dTarget = lateralTargets[li];
            double channelSafeSpeed =
                ComputeSafeSpeed(dTarget, current.s, clampedDesired, stopS, obstacles, numObs);
            static constexpr int MAX_SPEED_TARGETS = 4;
            double speedTargets[MAX_SPEED_TARGETS] = {
                channelSafeSpeed,
                channelSafeSpeed * 0.9,
                channelSafeSpeed * 0.75,
                0.0,
            };

            for (int vi = 0; vi < MAX_SPEED_TARGETS; ++vi)
            {
                double vTarget = Clamp(speedTargets[vi], 0.0, channelSafeSpeed);

                FrenetTrajectory frenet;
                if (!BuildLongitudinalQuartic(current.s, current.s_d, current.s_dd,
                                              vTarget, 0.0, T, frenet.sCoeffs))
                    continue;
                if (!BuildLateralQuintic(current.d, current.d_d, current.d_dd,
                                         dTarget, 0.0, 0.0, T, frenet.dCoeffs))
                    continue;

                frenet.T = T;
                frenet.valid = true;

                PlannedTrajectory trajectory;
                double cost = LARGE_NUMBER;
                if (!EvaluateTrajectory(frenet, current, clampedDesired, stopS,
                                        obstacles, numObs, roadLeftBound, roadRightBound,
                                        distToJunction, &cost, &trajectory))
                    continue;

                if (cost < best.candidate.cost)
                {
                    best.valid = true;
                    best.frenet = frenet;
                    best.frenet.cost = cost;
                    best.trajectory = trajectory;
                    best.candidate.dTarget = dTarget;
                    best.candidate.safeSpeed = trajectory.points[trajectory.numPoints - 1].speed;
                    best.candidate.cost = cost;
                    best.candidate.feasible = true;
                }
            }
        }
    }

    if (!best.valid)
    {
        CandidateResult fallback = Plan(current.d, clampedDesired, stopS, current.s,
                                        obstacles, numObs, roadLeftBound, roadRightBound,
                                        distToJunction);
        best.candidate = fallback;

        FrenetTrajectory frenet;
        double T = std::max(1.0, params.minTrajectoryTime);
        if (BuildLongitudinalQuartic(current.s, std::max(0.0, current.s_d), current.s_dd,
                                     fallback.feasible ? fallback.safeSpeed : 0.0, 0.0, T,
                                     frenet.sCoeffs)
            && BuildLateralQuintic(current.d, current.d_d, current.d_dd,
                                   Clamp(fallback.dTarget, roadRightBound, roadLeftBound),
                                   0.0, 0.0, T, frenet.dCoeffs))
        {
            frenet.T = T;
            frenet.valid = true;
            PlannedTrajectory trajectory;
            if (BuildPlannedTrajectory(frenet, roadLeftBound, roadRightBound, &trajectory))
            {
                best.valid = fallback.feasible;
                best.frenet = frenet;
                best.trajectory = trajectory;
            }
        }
    }

    return best;
}

}  // namespace rsim_driver
