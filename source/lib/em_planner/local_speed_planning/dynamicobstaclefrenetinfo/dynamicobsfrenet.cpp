#include "dynamicobsfrenet.hpp"

namespace rsim_driver
{
namespace
{

struct PerceptionReferencePoint
{
    double x = 0.0;
    double y = 0.0;
    double hdg = 0.0;
    double k = 0.0;
    double dk = 0.0;
    double s = 0.0;
};

std::vector<PerceptionReferencePoint> ToPerceptionReferenceLine(
    const localreferencelinepath& referenceLine)
{
    std::vector<PerceptionReferencePoint> converted;
    converted.reserve(referenceLine.size());
    for (const localreferencelinepoint& point : referenceLine)
    {
        converted.push_back({point.x,
                             point.y,
                             point.theta,
                             point.k,
                             point.dk,
                             point.s});
    }
    return converted;
}

}  // namespace

DynamicObstacleFrenetInfo::DynamicObstacleFrenetInfo(
    const FrenetObstaclePerceptionConfig& config)
    : perception_(config)
{
}

const FrenetObstaclePerceptionConfig& DynamicObstacleFrenetInfo::config() const
{
    return perception_.config();
}

void DynamicObstacleFrenetInfo::SetConfig(
    const FrenetObstaclePerceptionConfig& config)
{
    perception_.SetConfig(config);
}

const FrenetObstaclePerception& DynamicObstacleFrenetInfo::perception() const
{
    return perception_;
}

bool DynamicObstacleFrenetInfo::ConvertDynamicObstacles(
    const std::vector<rsim_plugin::ActorState>& actors,
    int32_t egoActorId,
    const localreferencelinepath& referenceLine,
    DynamicFrenetObstaclePerceptionResult* result) const
{
    const std::vector<PerceptionReferencePoint> perceptionReferenceLine =
        ToPerceptionReferenceLine(referenceLine);
    return perception_.ConvertDynamicObstacles(actors,
                                               egoActorId,
                                               perceptionReferenceLine,
                                               result);
}

}  // namespace rsim_driver
