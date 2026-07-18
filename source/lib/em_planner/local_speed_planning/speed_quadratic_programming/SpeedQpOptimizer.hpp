#pragma once

#include "DynamicSpeedPlanner.hpp"
#include "StDrivableArea.hpp"

#include <vector>

namespace rsim_driver
{

struct SpeedQpOptimizerConfig
{
    int num_points = 9;
    double dt = 1.0;
    double reference_speed = 13.0;
    double ego_length = 4.0;
    double longitudinal_safety_buffer = 0.5;
    double weight_reference_speed = 5.0;
    double weight_acceleration = 20.0;
    double weight_jerk = 1.0;
    double weight_progress = 1.0;
    double a_min = -5.0;
    double a_max = 3.0;
    int max_iterations = 1000;
};

struct SpeedQpResult
{
    bool qpsuccess = false;
    double objective = 0.0;
    std::vector<DynamicPlanSpeedPoint> stpoints;
};

class SpeedQpOptimizer
{
public:
    explicit SpeedQpOptimizer(const SpeedQpOptimizerConfig &config = {});

    const SpeedQpOptimizerConfig &config() const;
    void SetConfig(const SpeedQpOptimizerConfig &config);

    bool Optimize(const DynamicPlanSpeedPoint &start,
                  const StDrivableArea &drivable_area,
                  double reference_line_total_length,
                  SpeedQpResult *result) const;

private:
    SpeedQpOptimizerConfig config_;
};

}  // namespace rsim_driver
