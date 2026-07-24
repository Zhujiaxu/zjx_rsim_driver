#pragma once

#include "CartesianToFrenet.hpp"
#include "CollisionCost.hpp"
#include "FrenetObstaclePerception.hpp"

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
    int left_l_step_count = 11;
    int right_l_step_count = 11;

    double weight_l_prime = 1.0;
    double weight_l_double_prime = 1.0;
    double weight_ref_l = 1.0;
    double weight_collision = 60.0;

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
              const std::vector<StaticAndVirtualObsFrenetState>& obstacles,
              DpPlannerResult* result) const;

private:
    DpPlannerConfig config_;
};

}  // namespace rsim_driver
