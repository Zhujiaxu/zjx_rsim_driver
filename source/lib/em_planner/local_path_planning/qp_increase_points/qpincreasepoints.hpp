#pragma once

#include "DpPlanner.hpp"

#include <vector>

namespace rsim_driver
{

struct QpIncreasePointsConfig
{
    int count = 10;
};

class QpIncreasePoints
{
public:
    explicit QpIncreasePoints(QpIncreasePointsConfig config = {});

    void SetConfig(const QpIncreasePointsConfig& config);
    const QpIncreasePointsConfig& config() const;

    bool increasepoints(const std::vector<DpPathPoint>& qppath,
                        std::vector<DpPathPoint>* newqppath) const;

private:
    QpIncreasePointsConfig config_;
};

}  // namespace rsim_driver
