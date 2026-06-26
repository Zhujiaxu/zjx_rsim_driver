#pragma once

#include "PlanningStartPoint.hpp"
#include "CollisionCost.hpp"
#include "FrenetObstaclePerception.hpp"
#include "localpathreferline.hpp"
#include "computecutinandout.hpp"

#include <vector>

namespace rsim_driver
{

struct DynamicSpeedPlanConfig
{
    double time_step = 0.5;
    int time_step_count = 16;
    double s_step = 0.5;
    int s_step_count = 40;

    double reference_speed = 6.0;
    double weight_reference_speed = 5.0;
    double weight_acceleration = 1.0;
    double weight_jerk = 1.0;
    double weight_collision = 70.0;

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
struct DynamicSpeedPlanStartPoint
{
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
};

struct DynamicSpeedPlanResult
{
    bool dpsuccess = false;
    double total_cost = 0.0;
    std::vector<DynamicSpeedPoint> speed_points;
};
inline DynamicSpeedPlanStartPoint GetDynamicSpeedPlanStartPoint(PlanningStartResult startpoint)
{
    DynamicSpeedPlanStartPoint result;
    result.t = 0;
    result.s = 0;
    result.v = startpoint.start_point.speed;
    result.a = startpoint.start_point.accel;
    return result;
}

class DynamicSpeedPlanner
{
public:
    explicit DynamicSpeedPlanner(const DynamicSpeedPlanConfig& config = {});

    const DynamicSpeedPlanConfig& config() const;
    void SetConfig(const DynamicSpeedPlanConfig& config);

    bool Plan(const DynamicSpeedPlanStartPoint& start,
              const localreferencelinepath& reference_line,
              const DynamicFrenetObstaclePerceptionResult& dynamic_obstacles,
              DynamicSpeedPlanResult* result) const;

private:
    DynamicSpeedPlanConfig config_;
    ComputeCutInAndOut compute_cut_in_and_out_;
};

}  // namespace rsim_driver
