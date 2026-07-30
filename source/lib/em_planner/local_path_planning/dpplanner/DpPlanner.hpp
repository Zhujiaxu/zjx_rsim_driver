#pragma once

#include "CartesianToFrenet.hpp"
#include "CollisionCost.hpp"
#include "FrenetObstaclePerception.hpp"

#include <limits>
#include <vector>

namespace rsim_driver
{

enum class DpPlannerFallback
{
    Success,
    Other,
    Stop,
};

struct DpPlannerConfig
{
    double s_step = 0.4;
    int s_step_count = 70;
    double l_step = 0.3;
    int left_l_step_count = 15;
    int right_l_step_count = 15;

    double weight_l_prime = 5.0;
    double weight_l_double_prime = 1.0;
    double weight_ref_l = 3.0;
    double weight_collision = 100.0;

    StaticCollisionCostConfig collision;
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
    double total_cost = 0.0;
    DpPlannerFallback Flag = DpPlannerFallback::Success;
    std::vector<DpPathPoint> path;
};

class DpPlanner
{
public:
    explicit DpPlanner(const DpPlannerConfig& config = {});

    const DpPlannerConfig& config() const;
    void SetConfig(const DpPlannerConfig& config);

    bool Plan(const StartPointFrenetState& start,
              const std::vector<StaticObsFrenetState>& obstacles,
              DpPlannerResult* result,
              double max_s = std::numeric_limits<double>::infinity()) const;

private:
    DpPlannerConfig config_;
};

}  // namespace rsim_driver
