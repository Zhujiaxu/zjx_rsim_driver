#pragma once

#include "SpeedQpOptimizer.hpp"

#include <vector>

namespace rsim_driver
{

struct QpSpeedIncreasePointsConfig
{
    int count = 4;
};

class QpSpeedIncreasePoints
{
public:
    explicit QpSpeedIncreasePoints(QpSpeedIncreasePointsConfig config = {});

    void SetConfig(const QpSpeedIncreasePointsConfig& config);
    const QpSpeedIncreasePointsConfig& config() const;

    bool increasepoints(
        const QpSpeedOptimizerResult & speed_qp_result,
        std::vector<DynamicPlanSpeedPoint>* newqppointspath) const;

private:
    QpSpeedIncreasePointsConfig config_;
};

}  // namespace rsim_driver
