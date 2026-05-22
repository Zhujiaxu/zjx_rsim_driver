/*
 * EMPlanner — DP+QP staged local planner (Apollo EM Planner style).
 *
 * Pipeline: DP Path → QP Path → ST Graph → DP Speed → QP Speed → Trajectory.
 *
 * If referenceLine is provided, uses full pipeline with ReferenceLine-based
 * DP/QP.  Falls back to legacy SelectPath/SelectSpeed when referenceLine is
 * nullptr (backward compatible with RSimDriverPlugin until its next update).
 */
#pragma once

#include "DriverTypes.hpp"
#include "IPlanner.hpp"
#include "IPredictor.hpp"
#include "ReferenceLine.hpp"

#include <memory>

namespace rsim_driver
{

// --- Legacy structures (kept for backward compat) ---

enum class PathDecisionType
{
    KEEP_LANE, NUDGE_LEFT, NUDGE_RIGHT, BORROW_LEFT, BORROW_RIGHT, STOP_ONLY
};

enum class SpeedDecisionType
{
    CRUISE, FOLLOW, YIELD, STOP
};

struct ReferenceLineInfo
{
    int64_t roadId          = 0;
    int     laneId          = 0;
    double  laneWidth       = 3.5;
    double  leftBound       = 1.4;
    double  rightBound      = -1.4;
    bool    hasSameDirLeft  = false;
    bool    hasSameDirRight = false;
    double  distToJunction  = LARGE_NUMBER;
};

struct PathDecision
{
    PathDecisionType type    = PathDecisionType::KEEP_LANE;
    double           targetD = 0.0;
    double           cost    = LARGE_NUMBER;
};

struct SpeedDecision
{
    SpeedDecisionType type        = SpeedDecisionType::CRUISE;
    double            targetSpeed = 0.0;
    double            stopS       = LARGE_NUMBER;
    double            cost        = LARGE_NUMBER;
};

struct EMPlannerDebug
{
    double dpPathCost      = LARGE_NUMBER;
    double qpPathCost      = LARGE_NUMBER;
    double dpSpeedCost     = LARGE_NUMBER;
    double qpSpeedCost     = LARGE_NUMBER;
    double bestPathCost    = LARGE_NUMBER;   // legacy
    double bestSpeedCost   = LARGE_NUMBER;   // legacy
    char   failureReason[128] = {};
};

// --- Input / Output ---

struct EMPlannerInput
{
    FrenetState ego;
    double      desiredSpeed   = 10.0;
    double      simTime        = 0.0;
    double      planningDt     = 0.2;
    double      horizonTime    = 5.0;

    // Legacy: used when referenceLine is nullptr
    ReferenceLineInfo  reference;
    // New: full reference line for DP/QP pipeline
    const ReferenceLine* referenceLine = nullptr;

    const Obstacle* obstacles    = nullptr;
    int             numObstacles = 0;

    // New: obstacle predictor (nullptr → ConstantVelocityPredictor)
    const IPredictor* predictor = nullptr;
};

struct EMPlannerOutput
{
    bool              valid      = false;
    FrenetTrajectory  frenet;
    PlannedTrajectory trajectory;
    PathDecision      pathDecision;
    SpeedDecision     speedDecision;
    EMPlannerDebug    debug;

    // Convenience: extracted from path/speed decisions
    double targetSpeed = 0.0;
    double targetD     = 0.0;
};

// --- Parameters ---

    struct EMPlannerParams
    {
        double maxSpeed = 45.0;
        double maxAccel = 4.0;
        double maxDecel = 6.0;
        double maxLateralAccel = 2.5;
        double egoWidth = 2.0;
        double egoLength = 5.0;
        double safeGapLong = 6.0;
        double safeMarginLat = 0.5;
        double collisionLongBuffer = 1.0;
        double stopMargin = 1.5;
        double nudgeDistance = 0.8;
        double trajectoryTimeStep = 0.2;
        double junctionReturnDist = 50.0;
    };

    class EMPlanner : public IPlanner
    {
    public:
        EMPlannerParams params;

    // IPlanner interface.
    PlannerOutput Plan(const PlannerInput& input) override;

    // Full pipeline entry point (extended return with debug info).
    EMPlannerOutput Plan(const EMPlannerInput& input) const;

private:
    // Legacy path/speed selection (used when referenceLine == nullptr).
    PathDecision  SelectPath(const EMPlannerInput& input) const;
    SpeedDecision SelectSpeed(const EMPlannerInput& input, const PathDecision& path) const;
    bool BuildTrajectory(const EMPlannerInput& input,
                         const PathDecision& path,
                         const SpeedDecision& speed,
                         EMPlannerOutput* output) const;

    // New DP+QP pipeline (used when referenceLine != nullptr).
    bool PlanWithReferenceLine(const EMPlannerInput& input,
                               EMPlannerOutput* output) const;

    // Build lateral bounds for QP path from obstacles.
    void BuildLateralBounds(const ReferenceLine& refLine,
                            double egoS,
                            const Obstacle* obstacles,
                            int numObstacles,
                            const IPredictor& predictor,
                            const std::vector<double>& sGrid,
                            std::vector<double>* lb,
                            std::vector<double>* ub) const;
};

}  // namespace rsim_driver
