#pragma once

#include "CartesianToFrenet.hpp"
#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <utility>
#include <vector>

namespace rsim_driver
{

    enum class VirtualObstacleType
    {
        Normal,
        SlowLead,
        OncomingConflict,
    };

    struct VirtualObstacleSeed
    {
        int32_t source_actor_id = 0;
        VirtualObstacleType type = VirtualObstacleType::Normal;
        double longitudinal_buffer = 0.0;
        double lateral_buffer = 0.0;
        int ttl = 41;
    };

    struct StaticAndVirtualObsCartesianPoint
    {
        int32_t id = 0;
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        double length = 0.0;
        double width = 0.0;
    };

    struct DynamicObsCartesianPoint
    {
        int32_t id = 0;
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        double speed = 0.0;
        double length = 0.0;
        double width = 0.0;
    };

    struct StaticFrenetObstaclePerceptionResult
    {
        std::vector<StaticObsFrenetState> staticobstacles;
    };

    struct DynamicFrenetObstaclePerceptionResult
    {
        std::vector<DynamicObsFrenetState> dynamicobstacles;
    };

    struct VirtualFrenetObstaclePerceptionResult
    {
        std::vector<StaticObsFrenetState> virtualstaticobstacles;
    };

    struct FrenetObstaclePerceptionConfig
    {
        double static_speed_threshold = 1;
    };

    namespace frenet_obstacle_perception_detail
    {

        inline double ActorActualSpeed(const rsim_plugin::ActorState &actor)
        {
            const double velocitySpeed =
                std::sqrt(actor.vel_x * actor.vel_x + actor.vel_y * actor.vel_y);
            return std::max(std::fabs(actor.speed), velocitySpeed);
        }

        inline StaticAndVirtualObsCartesianPoint StaticAndVirtualObsToCartesianPoint(
            const rsim_plugin::ActorState &actor)
        {
            StaticAndVirtualObsCartesianPoint point;
            point.id = actor.id;
            point.x = actor.x;
            point.y = actor.y;
            point.heading = actor.h;
            point.length = actor.length;
            point.width = actor.width;
            return point;
        }
        inline DynamicObsCartesianPoint DynamicObsToCartesianPoint(
            const rsim_plugin::ActorState &actor)
        {
            DynamicObsCartesianPoint point;
            point.id = actor.id;
            point.x = actor.x;
            point.y = actor.y;
            point.heading = actor.h;
            point.speed = frenet_obstacle_perception_detail::ActorActualSpeed(actor);
            point.length = actor.length;
            point.width = actor.width;
            return point;
        }

    } // namespace frenet_obstacle_perception_detail

    class FrenetObstaclePerception
    {
    public:
        explicit FrenetObstaclePerception(
            const FrenetObstaclePerceptionConfig &config = {});

        const FrenetObstaclePerceptionConfig &config() const;
        void SetConfig(const FrenetObstaclePerceptionConfig &config);

        template <typename RefPointT>
        bool ConvertStaticObstacles(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const std::vector<RefPointT> &referencePoints,
            StaticFrenetObstaclePerceptionResult *result) const
        {
            if (result == nullptr || referencePoints.empty())
            {
                PluginLog("【perception】ConvertStaticObstacles: referencePoints is empty\n");
                return false;
            }
            result->staticobstacles.clear();
            StaticFrenetObstaclePerceptionResult converted;
            converted.staticobstacles.reserve(actors.size());

            const double staticSpeedThreshold =
                std::max(0.0, perceptionConfig_.static_speed_threshold);
            for (const rsim_plugin::ActorState &actor : actors)
            {
                if (actor.id == egoActorId)
                    continue;

                const double speed =
                    frenet_obstacle_perception_detail::ActorActualSpeed(actor);
                if (speed <= staticSpeedThreshold)
                {
                    StaticObsFrenetState obstacle;
                    if (!cartesian_to_frenet_detail ::StaticObsFrenetTransformer(referencePoints,
                                                                                 frenet_obstacle_perception_detail::StaticAndVirtualObsToCartesianPoint(actor),
                                                                                 &obstacle))
                    {
                        continue;
                    }
                    converted.staticobstacles.push_back(obstacle);
                }
            }

            *result = std::move(converted);
            return true;
        }

        template <typename RefPointT>
        bool ConvertDynamicObstacles(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const std::vector<RefPointT> &referencePoints,
            DynamicFrenetObstaclePerceptionResult *result) const
        {
            if (result == nullptr || referencePoints.empty())
            {
                PluginLog("【perception】ConvertDynamicObstacles: referencePoints is empty\n");
                return false;
            }
            result->dynamicobstacles.clear();
            DynamicFrenetObstaclePerceptionResult converted;
            converted.dynamicobstacles.reserve(actors.size());

            const double staticSpeedThreshold =
                std::max(0.0, perceptionConfig_.static_speed_threshold);
            for (const rsim_plugin::ActorState &actor : actors)
            {
                if (actor.id == egoActorId)
                    continue;

                const double speed =
                    frenet_obstacle_perception_detail::ActorActualSpeed(actor);
                if (speed > staticSpeedThreshold)
                {
                    DynamicObsFrenetState obstacle;
                    if (!cartesian_to_frenet_detail::DynamicObsFrenetTransformer(referencePoints,
                                                                                 frenet_obstacle_perception_detail::DynamicObsToCartesianPoint(actor),
                                                                                 &obstacle))
                    {
                        continue;
                    }
                    converted.dynamicobstacles.push_back(obstacle);
                }
            }

            *result = std::move(converted);
            return true;
        }

        /*template <typename RefPointT>
        bool ConvertVirtualObstacles(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const std::vector<RefPointT> &referencePoints,
            std::vector<VirtualObstacleSeed> &seeds,
            VirtualFrenetObstaclePerceptionResult *result) const
        {
            if (result == nullptr)
                return false;

            result->virtualstaticobstacles.clear();
            if (seeds.empty())
                return true;
            VirtualFrenetObstaclePerceptionResult converted;
            converted.virtualstaticobstacles.reserve(seeds.size());
            std::vector<VirtualObstacleSeed> aliveSeeds;
            aliveSeeds.reserve(seeds.size());
            for (VirtualObstacleSeed &seed : seeds)
            {
                const rsim_plugin::ActorState *sourceActor = nullptr;
                for (const rsim_plugin::ActorState &actor : actors)
                {
                    if (actor.id != egoActorId &&
                        actor.id == seed.source_actor_id)
                    {
                        sourceActor = &actor;
                        break;
                    }
                }
                if (sourceActor == nullptr)
                    continue;
                StaticAndVirtualObsFrenetState virtualObstacle;
                if (!cartesian_to_frenet_detail::VirtualObsFrenetTransformer(
                        referencePoints,
                        frenet_obstacle_perception_detail::StaticAndVirtualObsToCartesianPoint(*sourceActor),
                        seed,
                        &virtualObstacle))
                {
                    continue;
                }
                if (seed.ttl <= 0)
                {
                    PluginLog("[VOB-Resolve] id=%d DROPPED ttl=%d\n",
                              seed.source_actor_id, seed.ttl);
                    continue;
                }
                aliveSeeds.push_back(seed);
                PluginLog("[VOB-Resolve] id=%d ttl=%d s=%.2f l=%.2f len=%.2f "
                          "width=%.2f\n",
                          seed.source_actor_id, seed.ttl,
                          virtualObstacle.s, virtualObstacle.l,
                          virtualObstacle.length, virtualObstacle.width);
                converted.virtualstaticobstacles.push_back(virtualObstacle);
            }
            *result = std::move(converted);
            seeds = std::move(aliveSeeds);

            return true;
        }*/

    private:
        FrenetObstaclePerceptionConfig perceptionConfig_;
    };

} // namespace rsim_driver
