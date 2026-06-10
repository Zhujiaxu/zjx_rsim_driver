#include "perception/FrenetObstaclePerception.hpp"

namespace rsim_driver
{

FrenetObstaclePerception::FrenetObstaclePerception(
    const FrenetObstaclePerceptionConfig& config)
    : perceptionConfig_(config)
{
}

const FrenetObstaclePerceptionConfig& FrenetObstaclePerception::config() const
{
    return perceptionConfig_;
}

void FrenetObstaclePerception::SetConfig(
    const FrenetObstaclePerceptionConfig& config)
{
    perceptionConfig_ = config;
}

}  // namespace rsim_driver
