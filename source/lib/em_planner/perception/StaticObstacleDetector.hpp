#pragma once

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <cstdint>
#include <vector>

namespace rsim_driver
{

struct StaticObstacleConfig
{
    double max_static_speed = 0.1;
};

struct StaticObstacle
{
    int32_t id = 0;
    int32_t type = 0;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

std::vector<StaticObstacle> DetectStaticObstacles(
    const std::vector<rsim_plugin::ActorState>& actors,
    const std::vector<int32_t>& controlledActorIds,
    const StaticObstacleConfig& config = {});

}  // namespace rsim_driver
