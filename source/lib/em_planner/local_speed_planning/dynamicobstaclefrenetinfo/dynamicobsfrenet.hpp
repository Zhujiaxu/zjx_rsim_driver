#pragma once

#include "localpathreferline.hpp"
#include "perception/FrenetObstaclePerception.hpp"

#include <cstdint>
#include <vector>

namespace rsim_driver
{

class DynamicObstacleFrenetInfo
{
public:
    explicit DynamicObstacleFrenetInfo(
        const FrenetObstaclePerceptionConfig& config = {});

    const FrenetObstaclePerceptionConfig& config() const;
    void SetConfig(const FrenetObstaclePerceptionConfig& config);

    const FrenetObstaclePerception& perception() const;

    bool ConvertDynamicObstacles(
        const std::vector<rsim_plugin::ActorState>& actors,
        int32_t egoActorId,
        const localreferencelinepath& referenceLine,
        DynamicFrenetObstaclePerceptionResult* result) const;

private:
    FrenetObstaclePerception perception_;
};

}  // namespace rsim_driver
