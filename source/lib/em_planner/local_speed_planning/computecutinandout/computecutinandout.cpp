#include "computecutinandout.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_set>

namespace rsim_driver
{
    namespace
    {

        bool IsOppositeSign(double lhs, double rhs)
        {
            return (lhs < 0.0 && rhs > 0.0) || (lhs > 0.0 && rhs < 0.0);
        }
        bool HasPersistentLaneOverlap(
            const DynamicObsFrenetState &obstacle,
            const ComputeCutInAndOutConfig &config)
        {
            const double halfWidthL = std::max(0.0, config.half_buffers);
            const double obsHalfWidth = 0.5 * std::max(0.0, obstacle.width);
            return std::fabs(obstacle.l_dot) <= 0.5 &&
                   std::fabs(obstacle.l) - obsHalfWidth <= halfWidthL &&
                   obstacle.s > 0.0;
        }

        bool SameSeed(const VirtualObstacleSeed &lhs,
                      const VirtualObstacleSeed &rhs)
        {
            return lhs.source_actor_id == rhs.source_actor_id &&
                   lhs.type == rhs.type;
        }

        void AddUniqueSeed(const VirtualObstacleSeed &seed,
                           std::vector<VirtualObstacleSeed> *seeds)
        {
            if (seeds == nullptr)
                return;
            for (VirtualObstacleSeed &existing : *seeds)
            {
                if (SameSeed(existing, seed))
                {
                    existing = seed;
                    return;
                }
            }
            seeds->push_back(seed);
        }

        bool BuildVirtualObstacleSeed(
            const DynamicObsFrenetState &obstacle,
            const ComputeCutInAndOutConfig &config,
            const double &ego_speed,
            VirtualObstacleSeed *seed,
            std::unordered_map<int32_t, int> &slowCounters,
            std::unordered_map<int32_t, int> &oncomingCounters)
        {
            if (seed == nullptr)
                return false;

            if (!std::isfinite(obstacle.s_dot) ||
                !std::isfinite(ego_speed))
            {
                return false;
            }

            constexpr double kVirtualObstacleLateralBuffer = 0.2;
            double ratio = 10;

            // --- Slow lead virtual obstacle ---
            if (obstacle.s_dot > 0.0 &&
                obstacle.s_dot <= 3 &&
                HasPersistentLaneOverlap(obstacle, config))
            {
                int &slowCnt = slowCounters[obstacle.id];
                slowCnt++;

                /*PluginLog("[VOB-Slow] id=%d s=%.2f s_dot=%.2f l=%.2f ldot=%.2f "
                         "slowCnt=%d/%d\n",
                         obstacle.id, obstacle.s, obstacle.s_dot,
                         obstacle.l, obstacle.ldot, slowCnt, 20);*/

                VirtualObstacleSeed seedslow;
                seedslow.source_actor_id = obstacle.id;
                seedslow.type = VirtualObstacleType::SlowLead;
                seedslow.longitudinal_buffer =
                    obstacle.s_dot / ratio * obstacle.length;
                seedslow.lateral_buffer = kVirtualObstacleLateralBuffer;

                if (slowCnt > 20)
                {
                    slowCnt = 0;
                    *seed = seedslow;
                    return true;
                }
            }
            if(slowCounters.find(obstacle.id) != slowCounters.end())
            {
                slowCounters[obstacle.id] -= 2;
            }
            // Debug: log when an obstacle with low speed fails conditions
            /* if (state.s_dot > 0.0 && state.s_dot <= 3)
             {
                 PluginLog("[VOB-Fail] id=%d s=%.2f s_dot=%.2f l=%.2f ldot=%.2f "
                          "overlap=%d\n",
                          obstacle.id, state.s, state.s_dot,
                          state.l, state.ldot,
                          HasPersistentLaneOverlap(obstacle, config) ? 1 : 0);
             }*/

            /* --- Oncoming conflict virtual obstacle ---*/
            const double horizon = std::max(0.0, config.stationary_overlap_horizon);
            const double oncomingTravelTime =
                std::fabs(obstacle.s / obstacle.s_dot);
            if (obstacle.s_dot < 0.0 &&
                HasPersistentLaneOverlap(obstacle, config) &&
                oncomingTravelTime <= horizon)
            {
                int &oncomingCnt = oncomingCounters[obstacle.id];
                oncomingCnt++;

                VirtualObstacleSeed seedoncoming;
                seedoncoming.source_actor_id = obstacle.id;
                seedoncoming.type = VirtualObstacleType::OncomingConflict;
                seedoncoming.longitudinal_buffer =
                    std::fabs(2 * obstacle.s_dot / ratio * obstacle.length);
                seedoncoming.lateral_buffer = kVirtualObstacleLateralBuffer;

                if (oncomingCnt > 10)
                {
                    oncomingCnt = 0;
                    *seed = seedoncoming;
                    return true;
                }
            }
            if(oncomingCounters.find(obstacle.id) != oncomingCounters.end())
            {
                oncomingCounters[obstacle.id] -= 2;
            }
            return false;
            
        }

        std::optional<CutInAndOutInfo> ComputeObstacleCutInAndOut(
            const DynamicObsFrenetState &obstacle,
            const double &speedlinetotals,
            const ComputeCutInAndOutConfig &config)
        {
            const double halfWidthL = std::max(0.0, config.half_buffers);
            const double l_dotEpsilon = std::max(0.0, config.ldot_epsilon);
            const double stationaryHorizon =
                std::max(0.0, config.stationary_overlap_horizon);

            CutInAndOutInfo info;
            info.id = obstacle.id;

            if (std::fabs(obstacle.l_dot) <= l_dotEpsilon &&
                std::fabs(obstacle.l) - obstacle.width / 2 > halfWidthL)
            {
                return std::nullopt;
            }
            if (std::fabs(obstacle.l_dot) <= l_dotEpsilon)
            {
                info.tin = 0.0;
                info.tout = stationaryHorizon;
                info.sin = obstacle.s;
                info.sinmin = obstacle.s - obstacle.length / 2.0;
                info.sinmax = obstacle.s + obstacle.length / 2.0;
                info.sout = obstacle.s + obstacle.s_dot * stationaryHorizon > speedlinetotals
                                ? speedlinetotals
                                : obstacle.s + obstacle.s_dot * stationaryHorizon;
                info.soutmin = info.sout - obstacle.length / 2.0;
                info.soutmax = info.sout + obstacle.length / 2.0 >= speedlinetotals
                                   ? speedlinetotals
                                   : info.sout + obstacle.length / 2.0;
                return info;
            }
            const double leftBoundaryTime = (halfWidthL - obstacle.l) / obstacle.l_dot;
            const double rightBoundaryTime = (-halfWidthL - obstacle.l) / obstacle.l_dot;

            info.tin = std::min(leftBoundaryTime, rightBoundaryTime);
            info.tout = std::max(leftBoundaryTime, rightBoundaryTime);
            if (IsOppositeSign(info.tin, info.tout))
            {
                info.tin = 0.0;
                info.tout >= stationaryHorizon ? stationaryHorizon : info.tout;
            }

            if (info.tin < 0.0 && info.tout < 0.0)
            {

                return std::nullopt;
            }
            else
            {
                info.tin = (info.tin - obstacle.width / (2 * std::fabs(obstacle.l_dot))) <= 0.0
                               ? 0.0
                               : (std::fabs(obstacle.l) - 0.5 * std::fabs(obstacle.width) 
                               - halfWidthL) / std::fabs(obstacle.l_dot);
                info.tout = (info.tout + obstacle.width / (2 * std::fabs(obstacle.l_dot)))>= stationaryHorizon
                                ? stationaryHorizon
                                : info.tout+ obstacle.width / (2 * std::fabs(obstacle.l_dot));
                if (info.tin > stationaryHorizon)
                {
                    return std::nullopt;
                }

                info.sin = obstacle.s + info.tin * obstacle.s_dot;
                info.sinmin = info.sin - obstacle.length / 2.0;
                info.sinmax = info.sin + obstacle.length / 2.0;
                info.sout = (obstacle.s + info.tout * obstacle.s_dot)>= speedlinetotals
                                ? speedlinetotals
                                : obstacle.s + info.tout * obstacle.s_dot;
                info.soutmin = info.sout - obstacle.length / 2.0;
                info.soutmax = (info.sout + obstacle.length / 2.0)>= speedlinetotals
                                   ? speedlinetotals
                                   : info.sout + obstacle.length / 2.0;
            }

            return info;
        }

    } // namespace

    ComputeCutInAndOut::ComputeCutInAndOut(
        const ComputeCutInAndOutConfig &config)
        : config_(config)
    {
    }

    const ComputeCutInAndOutConfig &ComputeCutInAndOut::config() const
    {
        return config_;
    }

    void ComputeCutInAndOut::SetConfig(const ComputeCutInAndOutConfig &config)
    {
        config_ = config;
    }

    bool ComputeCutInAndOut::Compute(
        const std::vector<SpeedReferenceLinePoint> &referenceLine,
        const DynamicFrenetObstaclePerceptionResult &obstacles,
        const double &ego_speed,
        std::vector<CutInAndOutInfo> *result,
        std::vector<VirtualObstacleSeed> *seeds) const
    {
        if (result == nullptr || seeds == nullptr || referenceLine.empty() )
            return false;

        result->clear();
        result->reserve(obstacles.dynamicobstacles.size());
        for (const DynamicObsFrenetState &obstacle : obstacles.dynamicobstacles)
        {

            if (!std::isfinite(obstacle.s) || !std::isfinite(obstacle.s_dot) ||
                !std::isfinite(obstacle.l) || !std::isfinite(obstacle.l_dot) ||
                !std::isfinite(obstacle.length) || obstacle.length <= 0.0 ||
                !std::isfinite(obstacle.width) || obstacle.width <= 0.0)
            {
                continue;
            }
            if (obstacle.s < 0.0)
                continue;

            VirtualObstacleSeed seed;
            if (BuildVirtualObstacleSeed(obstacle,
                                         config_,
                                         ego_speed,
                                         &seed,
                                         slow_lead_counter_,
                                         oncoming_conflict_counter_))
            {
                AddUniqueSeed(seed, seeds);
                continue;
            }

            const std::optional<CutInAndOutInfo> boundary =
                ComputeObstacleCutInAndOut(obstacle,referenceLine.back().s, config_);
            if (boundary.has_value())
                result->push_back(*boundary);
        }

        // Prune counters for obstacles that no longer exist
        {
            std::unordered_set<int32_t> activeIds;
            for (const DynamicObsFrenetState &obs : obstacles.dynamicobstacles)
                activeIds.insert(obs.id);

            auto pruneMap =
                [&activeIds](std::unordered_map<int32_t, int> &m)
            {
                for (auto it = m.begin(); it != m.end();)
                {
                    if (activeIds.find(it->first) == activeIds.end())
                        it = m.erase(it);
                    else
                        ++it;
                }
            };
            pruneMap(slow_lead_counter_);
            pruneMap(oncoming_conflict_counter_);
        }

        return true;
    }

} // namespace rsim_driver
