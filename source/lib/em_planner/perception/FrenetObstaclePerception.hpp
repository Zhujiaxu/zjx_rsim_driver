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

    struct StaticFrenetObstacle
    {
        int32_t id = 0;
        double s = 0.0;
        double l = 0.0;
        double length = 0.0;
        double width = 0.0;
    };
    using DynamicFrenetState = CartesianFrenetState;
    struct DynamicFrenetObstacle
    {
        int32_t id = 0;
        DynamicFrenetState dynamicfrenetstate;
        double length = 0.0;
        double width = 0.0;
    };

    struct StaticFrenetObstaclePerceptionResult
    {
        std::vector<StaticFrenetObstacle> staticobstacles;
    };

    struct DynamicFrenetObstaclePerceptionResult
    {
        std::vector<DynamicFrenetObstacle> dynamicobstacles;
    };

    enum class VirtualObstacleType
    {
        SlowLead,
        OncomingConflict,
    };

    struct VirtualObstacleSeed
    {
        int32_t source_actor_id = 0;
        VirtualObstacleType type = VirtualObstacleType::SlowLead;
        double longitudinal_buffer = 0.0;
        double lateral_buffer = 0.0;
        int ttl = 41;
    };
    using VirtualFrenetObstacle = StaticFrenetObstacle;
    struct VirtualFrenetObstaclePerceptionResult
    {
        std::vector<VirtualFrenetObstacle> virtual_static_obstacles;
    };

    struct FrenetObstaclePerceptionConfig
    {
        double static_speed_threshold = 0.1;
    };

    namespace frenet_obstacle_perception_detail
    {

        struct ObstacleCartesianPoint
        {
            double x = 0.0;
            double y = 0.0;
            double heading = 0.0;
            double speed = 0.0;
            double accel = 0.0;
        };

        inline double ActorActualSpeed(const rsim_plugin::ActorState &actor)
        {
            const double velocitySpeed =
                std::sqrt(actor.vel_x * actor.vel_x + actor.vel_y * actor.vel_y);
            return std::max(std::fabs(actor.speed), velocitySpeed);
        }
        

    
        inline double PositiveOr(double value, double fallback)
        {
            return value > 0.0 ? value : fallback;
        }

        /*inline double ActorActualAccel(const rsim_plugin::ActorState &actor)
        {
            return std::sqrt(actor.acc_x * actor.acc_x + actor.acc_y * actor.acc_y);
        }*/

        template <typename RefPointT>
        StaticFrenetObstacle ToStaticFrenetObstacle(
            const rsim_plugin::ActorState &actor,
            const std::vector<RefPointT> &referencePoints)
        {
            const std::size_t matchIndex =
                FindMatchPointIndex(referencePoints, actor.x, actor.y);
            const RefPointT &matchedPoint = referencePoints[matchIndex];
            const RefPointT projectionPoint =
                FindProjectionPoint(referencePoints, actor.x, actor.y);

            const double tangentX = std::cos(matchedPoint.hdg);
            const double tangentY = std::sin(matchedPoint.hdg);
            const double normalX = -std::sin(matchedPoint.hdg);
            const double normalY = std::cos(matchedPoint.hdg);
            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double lateralDx = actor.x - projectionPoint.x;
            const double lateralDy = actor.y - projectionPoint.y;

            StaticFrenetObstacle obstacle;
            obstacle.id = actor.id;
            obstacle.s = matchedPoint.s +
                         projectionDx * tangentX +
                         projectionDy * tangentY;
            obstacle.l = lateralDx * normalX + lateralDy * normalY;
            obstacle.length = actor.length;
            obstacle.width = actor.width;
            return obstacle;
        }
        template <typename RefPointT>
        StaticFrenetObstacle ToVirtualFrenetObstacle(
            const rsim_plugin::ActorState &actor,
            const std::vector<RefPointT> &referencePoints,
            VirtualObstacleSeed &seed)
        {
            seed.ttl--;
            const std::size_t matchIndex =
                FindMatchPointIndex(referencePoints, actor.x, actor.y);
            const RefPointT &matchedPoint = referencePoints[matchIndex];
            const RefPointT projectionPoint =
                FindProjectionPoint(referencePoints, actor.x, actor.y);

            const double tangentX = std::cos(matchedPoint.hdg);
            const double tangentY = std::sin(matchedPoint.hdg);
            const double normalX = -std::sin(matchedPoint.hdg);
            const double normalY = std::cos(matchedPoint.hdg);
            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double lateralDx = actor.x - projectionPoint.x;
            const double lateralDy = actor.y - projectionPoint.y;

            StaticFrenetObstacle obstacle;
            obstacle.id = actor.id;
            obstacle.s = matchedPoint.s +
                         projectionDx * tangentX +
                         projectionDy * tangentY;
            obstacle.l = lateralDx * normalX + lateralDy * normalY;
            obstacle.length = actor.length + seed.longitudinal_buffer;
            obstacle.width = actor.width + seed.lateral_buffer;
            return obstacle;
        }

        inline ObstacleCartesianPoint ToObstacleCartesianPoint(
            const rsim_plugin::ActorState &actor)
        {
            ObstacleCartesianPoint point;
            point.x = actor.x;
            point.y = actor.y;
            point.heading = actor.h;
            point.speed = ActorActualSpeed(actor);
            point.accel = actor.acc_x;
            return point;
        }

        template <typename RefPointT>
        bool ActorToDynamicFrenetObstacle(
            const rsim_plugin::ActorState &actor,
            const std::vector<RefPointT> &referencePoints,
            DynamicFrenetObstacle *obstacle)
        {
            if (obstacle == nullptr || referencePoints.empty())
                return false;

            const ObstacleCartesianPoint point = ToObstacleCartesianPoint(actor);
            CartesianFrenetState frenet;
            if (!cartesian_to_frenet_detail::CartesianPointToFrenet(
                    referencePoints, point, 0.0, &frenet))
            {
                return false;
            }

            obstacle->id = actor.id;
            obstacle->dynamicfrenetstate = frenet;
            obstacle->length = actor.length;
            obstacle->width = actor.width;
            return true;
        }

        inline int32_t VirtualObstacleId(VirtualObstacleType type,
                                         int32_t sourceActorId)
        {
            const int32_t base =
                type == VirtualObstacleType::SlowLead ? -100000 : -200000;
            return base - sourceActorId;
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
                return false;

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
                    converted.staticobstacles.push_back(
                        frenet_obstacle_perception_detail::ToStaticFrenetObstacle(
                            actor, referencePoints));
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
                return false;

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
                if (speed <= staticSpeedThreshold)
                    continue;

                DynamicFrenetObstacle obstacle;
                if (!frenet_obstacle_perception_detail::ActorToDynamicFrenetObstacle(
                        actor, referencePoints, &obstacle))
                {
                    return false;
                }

                converted.dynamicobstacles.push_back(obstacle);
            }

            *result = std::move(converted);
            return true;
        }

        template <typename RefPointT>
        bool ConvertVirtualObstacles(
            const std::vector<rsim_plugin::ActorState> &actors,
            int32_t egoActorId,
            const std::vector<RefPointT> &referencePoints,
            std::vector<VirtualObstacleSeed> &seeds,
            VirtualFrenetObstaclePerceptionResult *result) const
        {
            if (result == nullptr || referencePoints.empty())
                return false;

            result->virtual_static_obstacles.clear();
            if (seeds.empty())
                return true;
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

                auto virtualObstacle =
                    frenet_obstacle_perception_detail::ToVirtualFrenetObstacle(
                        *sourceActor, referencePoints, seed);
                if (seed.ttl <= 0)
                {
                    /*std::fprintf(stderr,
                                 "[VOB-Resolve] id=%d DROPPED ttl=%d\n",
                                 seed.source_actor_id, seed.ttl);*/
                    continue;
                }
                aliveSeeds.push_back(seed);
                /*std::fprintf(stderr,
                             "[VOB-Resolve] id=%d ttl=%d s=%.2f l=%.2f len=%.2f "
                             "width=%.2f\n",
                             seed.source_actor_id, seed.ttl,
                             virtualObstacle.s, virtualObstacle.l,
                             virtualObstacle.length, virtualObstacle.width);*/
                result->virtual_static_obstacles.push_back(virtualObstacle);
            }
            seeds = std::move(aliveSeeds);

            return true;
        }

    private:
        FrenetObstaclePerceptionConfig perceptionConfig_;
    };

} // namespace rsim_driver
