/*
 * SamplingPlanner — Frenet sampling local trajectory planner.
 *
 * The legacy Plan() API still returns a single best lateral/speed candidate.
 * PlanTrajectory() generates short-horizon Frenet trajectories, filters them
 * for road bounds, dynamic limits and obstacle overlap, then returns the best
 * executable trajectory.
 */

#pragma once

#include "DriverTypes.hpp"
#include "IPlanner.hpp"

namespace rsim_driver
{
    struct CandidateResult
    {
        double dTarget   = 0.0;
        double safeSpeed = 0.0;
        double cost      = LARGE_NUMBER;
        bool   feasible  = false;
    };

    struct SamplingParams
    {
        double lateralStep        = 3.5;
        double lateralRange       = 7.0;
        double wRoute             = 1.0;
        double wSpeed             = 0.5;
        double wSmooth            = 0.05;
        double wTime              = 0.05;
        double wLateralChange     = 0.2;
        double wObstacleClearance = 5.0;
        double wJunction          = 10.0;
        double junctionReturnDist = 50.0;
        double safeGapLong        = 5.0;
        double safeMarginLat      = 0.5;
        double comfortDecel       = 2.0;
        double egoWidth           = 2.0;
        double egoLength          = 5.0;
        double minTrajectoryTime  = 2.0;
        double maxTrajectoryTime  = 5.0;
        double trajectoryTimeStep = 0.2;
        double sampleTimeStep     = 0.2;
        double maxSpeed           = 45.0;
        double maxAccel           = 4.0;
        double maxDecel           = 6.0;
        double maxJerk            = 8.0;
        double maxLateralAccel    = 2.5;
        double collisionLongBuffer = 1.0;
        double stopMargin         = 1.0;
    };

    struct TrajectoryPlanResult
    {
        CandidateResult candidate;
        FrenetTrajectory frenet;
        PlannedTrajectory trajectory;
        bool valid = false;
    };

    class SamplingPlanner : public IPlanner
    {
    public:
        SamplingParams params;

        // IPlanner interface.
        PlannerOutput Plan(const PlannerInput& input) override;

        CandidateResult Plan(double           currentD,
                             double           desiredSpeed,
                             double           stopS,
                             double           currentRefS,
                             const Obstacle*  obstacles,
                             int              numObstacles,
                             double           roadLeftBound,
                             double           roadRightBound,
                             double           distToJunction);

        TrajectoryPlanResult PlanTrajectory(const FrenetState& current,
                                            double             desiredSpeed,
                                            double             stopS,
                                            const Obstacle*    obstacles,
                                            int                numObstacles,
                                            double             roadLeftBound,
                                            double             roadRightBound,
                                            double             distToJunction);

    private:
        bool   IsPathBlocked(double dTarget, double currentRefS, const Obstacle* obstacles, int numObs);
        double ComputeSafeSpeed(double dTarget, double currentRefS, double desiredSpeed, double stopS,
                                const Obstacle* obstacles, int numObs);
        bool   BuildPlannedTrajectory(const FrenetTrajectory& frenet,
                                      double roadLeftBound,
                                      double roadRightBound,
                                      PlannedTrajectory* out) const;
        bool   EvaluateTrajectory(const FrenetTrajectory& frenet,
                                  const FrenetState& current,
                                  double desiredSpeed,
                                  double stopS,
                                  const Obstacle* obstacles,
                                  int numObs,
                                  double roadLeftBound,
                                  double roadRightBound,
                                  double distToJunction,
                                  double* cost,
                                  PlannedTrajectory* out) const;
    };

}  // namespace rsim_driver
