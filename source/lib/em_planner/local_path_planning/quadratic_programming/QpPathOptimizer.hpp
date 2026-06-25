#pragma once

#include "FrenetToCartesian.hpp"
#include "DrivableArea.hpp"

#include <utility>
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
    std::vector<CartesianPathPoint> localcartesianpath;
};

class QpPathOptimizer
{
public:
    explicit QpPathOptimizer(const QpPathOptimizerConfig& config = {});

    const QpPathOptimizerConfig& config() const;
    void SetConfig(const QpPathOptimizerConfig& config);

    template <typename RefPointT>
    bool Optimize(const CartesianFrenetState& start,
                  const std::vector<DpPathPoint>& coarsePath,
                  const DrivableArea& drivableArea,
                  const std::vector<StaticFrenetObstacle>& staticObstacles,
                  const std::vector<RefPointT>& referencePoints,
                  std::vector<DpPathPoint>* localfrenetpath,
                  QpPathResult* result) const;

private:
    bool OptimizeFrenet(
        const CartesianFrenetState& start,
        const std::vector<DpPathPoint>& coarsePath,
        const DrivableArea& drivableArea,
        const std::vector<StaticFrenetObstacle>& staticObstacles,
        std::vector<DpPathPoint>* localfrenetpath,
        double* objective) const;

    QpPathOptimizerConfig config_;
};

template <typename RefPointT>
bool QpPathOptimizer::Optimize(
    const CartesianFrenetState& start,
    const std::vector<DpPathPoint>& coarsePath,
    const DrivableArea& drivableArea,
    const std::vector<StaticFrenetObstacle>& staticObstacles,
    const std::vector<RefPointT>& referencePoints,
    std::vector<DpPathPoint>* localfrenetpath,
    QpPathResult* result) const
{
    if (localfrenetpath == nullptr || result == nullptr)
        return false;

    QpPathResult output;
    std::vector<DpPathPoint> optimizedFrenetPath;
    double objective = 0.0;
    if (!OptimizeFrenet(start,
                        coarsePath,
                        drivableArea,
                        staticObstacles,
                        &optimizedFrenetPath,
                        &objective))
    {
        localfrenetpath->clear();
        *result = output;
        return false;
    }

    if (!FrenetPathToCartesian(referencePoints,
                               optimizedFrenetPath,
                               &output.localcartesianpath))
    {
        localfrenetpath->clear();
        *result = output;
        return false;
    }

    output.qpsuccess = true;
    output.objective = objective;
    *localfrenetpath = std::move(optimizedFrenetPath);
    *result = std::move(output);
    return true;
}

}  // namespace rsim_driver
