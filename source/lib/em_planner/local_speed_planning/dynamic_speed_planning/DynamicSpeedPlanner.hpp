#pragma once

#include "PlanningStartPoint.hpp"
#include "CollisionCost.hpp"
#include "FrenetObstaclePerception.hpp"
#include "localpathreferline.hpp"
#include "computecutinandout.hpp"

#include <vector>

namespace rsim_driver
{

struct DynamicPlanSpeedConfig
{
    double time_step = 0.2;
    int time_step_count = 40;
    double planning_period = 0.05;
    double s_step = 0.2;
    int s_step_count = 140;

    double reference_speed = 13.0;
    double weight_reference_speed = 5.0;
    double weight_acceleration = 20.0;
    double weight_jerk = 1.0;
    double weight_collision = 70.0;

    DynamicCollisionCostConfig collisionconfig;
};

struct DynamicPlanSpeedPoint
{
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
    //double jerk = 0.0;
    //int rowindex = -1;
};

struct DynamicPlanSpeedResult
{
    bool dpsuccess = false;
    double total_cost = 0.0;
    std::vector<DynamicPlanSpeedPoint> stpoints;
};
inline DynamicPlanSpeedPoint GetDynamicSpeedPlanStartPoint(PlanningStartResult startpoint)
{
    DynamicPlanSpeedPoint result;
    result.t = 0;
    result.s = 0;
    result.v = startpoint.start_point.speed;
    result.a = startpoint.start_point.accel;
    return result;
}

class DynamicPlanSpeedPlanner
{
public:
    explicit DynamicPlanSpeedPlanner(const DynamicPlanSpeedConfig& config = {});

    const DynamicPlanSpeedConfig& config() const;
    void SetConfig(const DynamicPlanSpeedConfig& config);

    bool Plan(const DynamicPlanSpeedPoint& start,
              const localreferencelinepath& reference_line,
              const std::vector<CutInAndOutInfo>& STBoundaryInfos,
              DynamicPlanSpeedResult* result) const;

private:
    DynamicPlanSpeedConfig config_;
};

}  // namespace rsim_driver
