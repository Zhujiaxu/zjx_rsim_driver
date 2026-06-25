#pragma once

#include "PlanningStartPoint.hpp"
#include "CollisionCost.hpp"
#include "FrenetObstaclePerception.hpp"

#include <vector>

namespace rsim_driver
{

struct DynamicSpeedPlanConfig
{
    double time_step = 0.5;
    int time_step_count = 16;
    double s_step = 0.5;
    int s_step_count = 40;

    double reference_speed = 10.0;
    double weight_reference_speed = 1.0;
    double weight_acceleration = 1.0;
    double weight_jerk = 1.0;
    double weight_collision = 30.0;

    DynamicCollisionCostConfig collision;
};

struct DynamicSpeedPoint
{
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
    double jerk = 0.0;
    int rowindex = -1;
};

struct DynamicSpeedPlanResult
{
    bool dpsuccess = false;
    double total_cost = 0.0;
    std::vector<DynamicSpeedPoint> speed_points;
};

class DynamicSpeedPlanner
{
public:
    explicit DynamicSpeedPlanner(const DynamicSpeedPlanConfig& config = {});

    const DynamicSpeedPlanConfig& config() const;
    void SetConfig(const DynamicSpeedPlanConfig& config);

    bool Plan(const PlanningStartResult& start,
              double path_length,
              const DynamicFrenetObstaclePerceptionResult& dynamic_obstacles,
              DynamicSpeedPlanResult* result) const;

private:
    DynamicSpeedPlanConfig config_;
};

}  // namespace rsim_driver
