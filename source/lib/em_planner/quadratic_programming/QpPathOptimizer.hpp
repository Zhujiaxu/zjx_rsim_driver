#pragma once

#include "drivable_area/DrivableArea.hpp"

#include <vector>

namespace rsim_driver
{

struct QpPathOptimizerConfig
{
    double weight_reference_l = 0.5;
    double weight_smooth_l_prime = 1.0;
    double weight_smooth_l_double_prime = 1.0;
    double weight_jerk = 1.0;
    double weight_collision = 80.0;
    double weight_drivable_area_center = 3.0;
    double collision_lateral_buffer = 1.3;
    int max_iterations = 1000;
};

struct QpPathResult
{
    bool qpsuccess = false;
    double objective = 0.0;
    std::vector<DpPathPoint> path;
};

class QpPathOptimizer
{
public:
    explicit QpPathOptimizer(const QpPathOptimizerConfig& config = {});

    const QpPathOptimizerConfig& config() const;
    void SetConfig(const QpPathOptimizerConfig& config);

    bool Optimize(const CartesianFrenetState& start,
                  const std::vector<DpPathPoint>& coarsePath,
                  const DrivableArea& drivableArea,
                  const std::vector<StaticFrenetObstacle>& staticObstacles,
                  QpPathResult* result) const;

private:
    QpPathOptimizerConfig config_;
};

}  // namespace rsim_driver
