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
    int s_step_count = 40;
    double l_step = 0.4;
    int left_l_step_count = 8;
    int right_l_step_count = 8;

    double weight_l_prime = 1.0;
    double weight_l_double_prime = 1.0;
    double weight_ref_l = 1.0;
    double weight_collision = 30.0;

    CollisionCostConfig collision;
};

struct DpPathPoint
{
    double s = 0.0;
    double l = 0.0;
    double l_prime = 0.0;
    double l_double_prime = 0.0;
};

struct DpPlannerResult
{
    bool dpsuccess = false;
    double total_cost = 0.0;
    std::vector<DpPathPoint> path;
};

class DpPlanner
{
public:
    explicit DpPlanner(const DpPlannerConfig& config = {});

    const DpPlannerConfig& config() const;
    void SetConfig(const DpPlannerConfig& config);

    bool Plan(const CartesianFrenetState& start,
              const std::vector<StaticFrenetObstacle>& obstacles,
              DpPlannerResult* result) const;

private:
    DpPlannerConfig config_;
};

}  // namespace rsim_driver
