#pragma once

#include "CartesianToFrenet.hpp"
#include "planning_start/PlanningStartPoint.hpp"
#include "planning_start_sl/PlanningStartFrenetState.hpp"

#include <vector>

namespace rsim_driver
{

template <typename RefPointT>
bool ComputePlanningStartSl(const PlanningStartPoint& planningStart,
                            const std::vector<RefPointT>& referencePoints,
                            PlanningStartFrenetState* frenetState)
{
    if (frenetState == nullptr)
        return false;

    CartesianFrenetState cartesianFrenet;
    if (!CartesianToFrenet(referencePoints, planningStart, &cartesianFrenet))
        return false;

    PlanningStartFrenetState state;
    state.s = cartesianFrenet.s;
    state.s_dot = cartesianFrenet.s_dot;
    state.s_ddot = cartesianFrenet.s_ddot;
    state.l = cartesianFrenet.l;
    state.l_prime = cartesianFrenet.l_prime;
    state.l_double_prime = cartesianFrenet.l_double_prime;

    *frenetState = state;
    return true;
}

}  // namespace rsim_driver
