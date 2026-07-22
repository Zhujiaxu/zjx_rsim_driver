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
            const DynamicFrenetObstacle &obstacle,
            const ComputeCutInAndOutConfig &config)
        {
            const DynamicFrenetState &state = obstacle.dynamicfrenetstate;
            const double halfWidthL = std::max(0.0, config.half_vehicle_width_l);
            const double obsHalfWidth = 0.5 * std::max(0.0, obstacle.width);
            return std::fabs(state.l_dot) <= 0.3 &&
                   std::fabs(state.l) - obsHalfWidth <= halfWidthL &&
                   state.s > 0.0;
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
            const DynamicFrenetObstacle &obstacle,
            const ComputeCutInAndOutConfig &config,
            double egoSDot,
            double planningPeriod,
            double tPlan,
            VirtualObstacleSeed *seed,
            std::unordered_map<int32_t, int> &slowCounters,
            std::unordered_map<int32_t, int> &oncomingCounters)
        {
            if (seed == nullptr)
                return false;

            const DynamicFrenetState &state = obstacle.dynamicfrenetstate;
            if (!std::isfinite(state.s_dot) ||
                !std::isfinite(egoSDot) ||
                !std::isfinite(planningPeriod) ||
                !std::isfinite(tPlan))
            {
                return false;
            }

            constexpr double kVirtualObstacleLateralBuffer = 0;
            double ratio = 10;

            // --- Slow lead virtual obstacle ---
            // Require s_dot > 0 so oncoming traffic (s_dot < 0) falls through
            // to the OncomingConflict branch below.
            if (state.s_dot > 0.0 &&
                state.s_dot <= 3 &&
                HasPersistentLaneOverlap(obstacle, config))
            {
                int &slowCnt = slowCounters[obstacle.id];
                slowCnt++;

                /*std::fprintf(stderr,
                             "[VOB-Slow] id=%d s=%.2f s_dot=%.2f l=%.2f ldot=%.2f "
                             "slowCnt=%d/%d\n",
                             obstacle.id, state.s, state.s_dot,
                             state.l, state.ldot, slowCnt, 20);*/

                VirtualObstacleSeed seedslow;
                seedslow.source_actor_id = obstacle.id;
                seedslow.type = VirtualObstacleType::SlowLead;
                seedslow.longitudinal_buffer =
                    state.s_dot / ratio * obstacle.length;
                seedslow.lateral_buffer = kVirtualObstacleLateralBuffer;

                if (slowCnt > 20)
                {
                    slowCnt = 0;
                    *seed = seedslow;
                    return true;
                }
                // Obstacle is classified as slow lead — reset oncoming counter
                oncomingCounters[obstacle.id] = 0;
                return false;
            }
            // Slow lead condition not met: reset counter (consecutive requirement)
            slowCounters[obstacle.id] = 0;

            // Debug: log when an obstacle with low speed fails conditions
           /* if (state.s_dot > 0.0 && state.s_dot <= 3)
            {
                std::fprintf(stderr,
                             "[VOB-Fail] id=%d s=%.2f s_dot=%.2f l=%.2f ldot=%.2f "
                             "overlap=%d\n",
                             obstacle.id, state.s, state.s_dot,
                             state.l, state.ldot,
                             HasPersistentLaneOverlap(obstacle, config) ? 1 : 0);
            }*/

            // --- Oncoming conflict virtual obstacle ---
            const double horizon = std::max(0.0, tPlan);
            const double oncomingTravelTime =
                std::fabs(state.s / state.s_dot);
            if (state.s_dot < 0.0 &&
                HasPersistentLaneOverlap(obstacle, config) &&
                oncomingTravelTime <= horizon)
            {
                int &oncomingCnt = oncomingCounters[obstacle.id];
                oncomingCnt++;

                VirtualObstacleSeed seedoncoming;
                seedoncoming.source_actor_id = obstacle.id;
                seedoncoming.type = VirtualObstacleType::OncomingConflict;
                seedoncoming.longitudinal_buffer =
                    -2 * state.s_dot / ratio * obstacle.length;
                seedoncoming.lateral_buffer = kVirtualObstacleLateralBuffer;

                if (oncomingCnt > 10)
                {
                    oncomingCnt = 0;
                    *seed = seedoncoming;
                    return true;
                }
                return false;
            }
            // Oncoming condition not met: reset counter (consecutive requirement)
            oncomingCounters[obstacle.id] = 0;

            return false;
        }

        std::optional<CutInAndOutInfo> ComputeObstacleCutInAndOut(
            const DynamicFrenetObstacle &obstacle,
            const ComputeCutInAndOutConfig &config)
        {
            const double halfWidthL = std::max(0.0, config.half_vehicle_width_l);
            const double l_dotEpsilon = std::max(0.0, config.ldot_epsilon);
            const double stationaryHorizon =
                std::max(0.0, config.stationary_overlap_horizon);
            const DynamicFrenetState &state = obstacle.dynamicfrenetstate;

            CutInAndOutInfo info;
            info.id = obstacle.id;

            /*if (std::fabs(state.l_dot) <= l_dotEpsilon &&
                std::fabs(state.l) > halfWidthL)
            {
                return std::nullopt;
            }
            if (std::fabs(state.l_dot) <= l_dotEpsilon)
            {
                info.tin = 0.0;
                info.tout = stationaryHorizon;
                info.sin = state.s;
                info.sinmin = state.s - obstacle.length / 2.0;
                info.sinmax = state.s + obstacle.length / 2.0;
                info.sout = state.s + state.s_dot * stationaryHorizon;
                info.soutmin = info.sout - obstacle.length / 2.0;
                info.soutmax = info.sout + obstacle.length / 2.0;
                return info;
            }*/

            const double leftBoundaryTime = (halfWidthL - state.l) / state.l_dot;
            const double rightBoundaryTime = (-halfWidthL - state.l) / state.l_dot;

            info.tin = std::min(leftBoundaryTime, rightBoundaryTime);
            info.tout = std::max(leftBoundaryTime, rightBoundaryTime);
            if (IsOppositeSign(info.tin, info.tout))
            {
                info.tin = 0.0;
                info.tout >=stationaryHorizon ? stationaryHorizon : info.tout;
            }

            if (info.tin < 0.0 && info.tout < 0.0)
            {

                return std::nullopt;
            }
            else
            {
                if(info.tin > stationaryHorizon)
                {
                    return std::nullopt;
                }
                if(info.tout >= stationaryHorizon)
                {
                    info.tout = stationaryHorizon;
                }
                info.sin = state.s + info.tin * state.s_dot;
                info.sinmin = info.sin - obstacle.length / 2.0;
                info.sinmax = info.sin + obstacle.length / 2.0;
                info.sout = state.s + info.tout * state.s_dot;
                info.soutmin = info.sout - obstacle.length / 2.0;
                info.soutmax = info.sout + obstacle.length / 2.0;
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
        const localreferencelinepath &referenceLine,
        const DynamicFrenetObstaclePerceptionResult &obstacles,
        double ego_s_dot,
        double planningPeriod,
        double t_plan,
        std::vector<CutInAndOutInfo> *result,
        std::vector<VirtualObstacleSeed> *seeds) const
    {
        if (result == nullptr || seeds == nullptr || referenceLine.size() < 2)
            return false;

        result->clear();
        result->reserve(obstacles.dynamicobstacles.size());
        for (const DynamicFrenetObstacle &obstacle : obstacles.dynamicobstacles)
        {
            const DynamicFrenetState &state = obstacle.dynamicfrenetstate;
            if (!std::isfinite(state.s) || !std::isfinite(state.s_dot) ||
                !std::isfinite(state.l) || !std::isfinite(state.l_dot) ||
                !std::isfinite(obstacle.length) || obstacle.length <= 0.0 ||
                !std::isfinite(obstacle.width) || obstacle.width <= 0.0)
            {
                continue;
            }
            if (obstacle.dynamicfrenetstate.s < 0.0)
                continue;

            VirtualObstacleSeed seed;
            if (BuildVirtualObstacleSeed(obstacle,
                                         config_,
                                         ego_s_dot,
                                         planningPeriod,
                                         t_plan,
                                         &seed,
                                         slow_lead_counter_,
                                         oncoming_conflict_counter_))
            {
                AddUniqueSeed(seed, seeds);
                continue;
            }

            const std::optional<CutInAndOutInfo> boundary =
                ComputeObstacleCutInAndOut(obstacle, config_);
            if (boundary.has_value())
                result->push_back(*boundary);
        }

        // Prune counters for obstacles that no longer exist
        {
            std::unordered_set<int32_t> activeIds;
            for (const DynamicFrenetObstacle &obs : obstacles.dynamicobstacles)
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
