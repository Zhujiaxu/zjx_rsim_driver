/*
 * IPlanner — common planner interface.
 *
 * All planners (EMPlanner, SamplingPlanner) implement this interface so that
 * RSimDriverPlugin can select a planner via polymorphism.
 */
#pragma once

#include "DriverTypes.hpp"

namespace rsim_driver
{

struct ReferenceLine;  // forward declaration; full include needed for users

struct PlannerInput
{
    FrenetState ego;
    double      desiredSpeed  = 10.0;
    double      simTime       = 0.0;
    double      planningDt    = 0.2;
    double      horizonTime   = 5.0;

    const ReferenceLine* referenceLine = nullptr;
    const Obstacle*      obstacles     = nullptr;
    int                  numObstacles  = 0;
};

struct PlannerOutput
{
    bool              valid      = false;
    FrenetTrajectory  frenet;
    PlannedTrajectory trajectory;
};

class IPlanner
{
public:
    virtual ~IPlanner() = default;
    virtual PlannerOutput Plan(const PlannerInput& input) = 0;
};

}  // namespace rsim_driver
