#pragma once

#include "QpPathOptimizer.hpp"
#include <vector>

namespace rsim_driver
{

struct QpIncreasePointsConfig
{
    int count = 10;
};
struct QpIncreasePointsResult
{
    std::vector<DpPathPoint> localfrenetpath;
};

class QpIncreasePoints
{
public:
    explicit QpIncreasePoints(QpIncreasePointsConfig config = {});

    void SetConfig(const QpIncreasePointsConfig& config);
    const QpIncreasePointsConfig& config() const;

    bool increasepoints(const QpPathResult & qppath,
                        QpIncreasePointsResult* newqppathresult) const;

private:
    QpIncreasePointsConfig config_;
};

}  // namespace rsim_driver
