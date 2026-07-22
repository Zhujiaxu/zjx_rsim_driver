#pragma once

#include "FrenetToCartesian.hpp"
#include "DrivableArea.hpp"

#include <utility>
#include <vector>

namespace rsim_driver
{

    struct QpPathOptimizerConfig
    {
        int num_points = 29;
        double ds = 1.0;
        double ego_length = 4.0;
        double ego_width = 2.0;
        double weight_reference_l = 0.5;
        double weight_smooth_l_prime = 1.0;
        double weight_smooth_l_double_prime = 1.0;
        double weight_jerk = 1.0;
        double weight_drivable_area_center = 3.0;
        int max_iterations = 1000;
    };
    enum class QpPathOptimizerFallback
    {
        Success,
        Other,
        SolveFailStop,
    };
    struct QpPathResult
    {
        QpPathOptimizerFallback Flag = QpPathOptimizerFallback::Success;
        double objective = 0.0;
        std::vector<DpPathPoint> localfrenetpath;
    };

    class QpPathOptimizer
    {
    public:
        explicit QpPathOptimizer(const QpPathOptimizerConfig &config = {});

        const QpPathOptimizerConfig &config() const;
        void SetConfig(const QpPathOptimizerConfig &config);
        bool Optimize(
            const CartesianFrenetState &start,
            const DrivableAreaResult &drivableArea,
            QpPathResult *result) const;

    private:
        QpPathOptimizerConfig config_;
    };

} // namespace rsim_driver
