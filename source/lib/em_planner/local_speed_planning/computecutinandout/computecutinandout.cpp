#include "computecutinandout.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rsim_driver
{
namespace
{

double PositiveInfinity()
{
    return std::numeric_limits<double>::infinity();
}

bool IsOppositeSign(double lhs, double rhs)
{
    return (lhs < 0.0 && rhs > 0.0) || (lhs > 0.0 && rhs < 0.0);
}

CutInAndOutInfo ComputeObstacleCutInAndOut(
    const DynamicFrenetObstacle& obstacle,
    const ComputeCutInAndOutConfig& config)
{
    const double halfWidthL = std::max(0.0, config.half_vehicle_width_l);
    const double ldotEpsilon = std::max(0.0, config.ldot_epsilon);
    const DynamicFrenetState& state = obstacle.dynamicfrenetstate;

    CutInAndOutInfo info;
    info.id = obstacle.id;

    if (std::fabs(state.ldot) <= ldotEpsilon)
    {
        info.tin = PositiveInfinity();
        info.tout = PositiveInfinity();
        info.sin = PositiveInfinity();
        info.sout = PositiveInfinity();
        return info;
    }

    const double leftBoundaryTime = (halfWidthL - state.l) / state.ldot;
    const double rightBoundaryTime = (-halfWidthL - state.l) / state.ldot;

    info.tin = std::min(leftBoundaryTime, rightBoundaryTime);
    info.tout = std::max(leftBoundaryTime, rightBoundaryTime);
    if (IsOppositeSign(info.tin, info.tout))
        info.tin = 0.0;

    if (info.tin < 0.0 && info.tout < 0.0)
    {
        info.sin = PositiveInfinity();
        info.sout = PositiveInfinity();
    }
    else
    {
        info.sin = state.s + info.tin * state.s_dot;
        info.sout = state.s + info.tout * state.s_dot;
    }

    return info;
}

}  // namespace

ComputeCutInAndOut::ComputeCutInAndOut(
    const ComputeCutInAndOutConfig& config)
    : config_(config)
{
}

const ComputeCutInAndOutConfig& ComputeCutInAndOut::config() const
{
    return config_;
}

void ComputeCutInAndOut::SetConfig(const ComputeCutInAndOutConfig& config)
{
    config_ = config;
}

bool ComputeCutInAndOut::Compute(
    const localreferencelinepath& referenceLine,
    const DynamicFrenetObstaclePerceptionResult& obstacles,
    std::vector<CutInAndOutInfo>* result) const
{
    if (result == nullptr || referenceLine.empty())
        return false;

    result->clear();
    result->reserve(obstacles.dynamicobstacles.size());
    for (const DynamicFrenetObstacle& obstacle : obstacles.dynamicobstacles)
    {
        result->push_back(ComputeObstacleCutInAndOut(obstacle, config_));
    }

    return true;
}

}  // namespace rsim_driver
