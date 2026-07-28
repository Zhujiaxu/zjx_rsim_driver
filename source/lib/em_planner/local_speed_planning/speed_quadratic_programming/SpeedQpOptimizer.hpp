#pragma once

#include "DynamicSpeedPlanner.hpp"
#include "StDrivableArea.hpp"

#include <vector>

namespace rsim_driver
{

    struct QpSpeedOptimizerConfig
    {
        double dt = 0.8;
        double dqpt=0.2;
        double reference_speed = 13.0;
        double ego_length = 4.0;
        double longitudinal_safety_buffer = 0.5;
        double weight_reference_speed = 5.0;
        double weight_acceleration = 20.0;
        double weight_jerk = 1.0;
        double weight_progress = 1.0;
        double a_min = -5.0;
        double a_max = 8.0;
        int max_iterations = 1000;
    };
    enum class QpSpeedOptimizerFallback
    {
        Success,
        Other,
        Stop,
    };
    struct QpSpeedOptimizerResult
    {
        QpSpeedOptimizerFallback Flag = QpSpeedOptimizerFallback::Success;
        double objective = 0.0;
        std::vector<DynamicPlanSpeedPoint> stpoints;
    };

    class SpeedQpOptimizer
    {
    public:
        explicit SpeedQpOptimizer(const QpSpeedOptimizerConfig &config = {});

        const QpSpeedOptimizerConfig &config() const;
        void SetConfig(const QpSpeedOptimizerConfig &config);

        bool Optimize(const DynamicPlanSpeedPoint &start,
                      const StDrivableAreaResult &drivable_area,
                      QpSpeedOptimizerResult *result) const;

    private:
        QpSpeedOptimizerConfig config_;
    };

} // namespace rsim_driver
