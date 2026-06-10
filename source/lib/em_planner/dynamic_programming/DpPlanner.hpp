#pragma once

#include "CartesianToFrenet.hpp"
#include "CollisionCost.hpp"
#include "perception/FrenetObstaclePerception.hpp"

#include <vector>

namespace rsim_driver
{

struct DpPlannerConfig
{
    double s_step = 0.5;
    double total_length = 20.0;
    double l_step = 0.4;
    double left_width = 3.0;
    double right_width = 3.0;

    double weight_l_prime = 1.0;
    double weight_l_double_prime = 1.0;
    double weight_ref_l = 1.0;
    double weight_collision = 1.0;

    CollisionCostConfig collision;
};

struct DpPathPoint
{
    double s = 0.0;
    double l = 0.0;
};

struct DpPlannerResult
{
    bool dpsuccess = false;
    double total_cost = 0.0;
    std::vector<DpPathPoint> path;
};

bool Plan(const CartesianFrenetState& start,
          const std::vector<StaticFrenetObstacle>& obstacles,
          const DpPlannerConfig& config,
          DpPlannerResult* result);

}  // namespace rsim_driver
