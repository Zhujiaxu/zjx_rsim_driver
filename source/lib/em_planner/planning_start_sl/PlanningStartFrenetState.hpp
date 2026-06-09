#pragma once

namespace rsim_driver
{

struct PlanningStartFrenetState
{
    double s = 0.0;
    double s_dot = 0.0;
    double s_ddot = 0.0;
    double l = 0.0;
    double l_prime = 0.0;
    double l_double_prime = 0.0;
};

}  // namespace rsim_driver
