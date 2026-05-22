/*
 * PerceptionHelper — implementation (Q0: same-road scan).
 * Cross-road scanning is handled by the plugin layer (RSimDriverPlugin).
 */
#include "PerceptionHelper.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

ScanResult PerceptionHelper::Scan(const ActorState&              ego,
                                  const std::vector<ActorState>& actors,
                                  double                         lateralDist,
                                  double                         lookahead,
                                  double                         egoLaneWidth) const
{
    ScanResult result;

    for (size_t i = 0; i < actors.size(); ++i)
    {
        const ActorState& other = actors[i];
        if (other.id == ego.id)
            continue;

        // 120° threshold allows crossing traffic (e.g. bicycles at ~90°).
        if (GetAbsAngleDifference(ego.h, other.h) > M_PI * 2.0 / 3.0)
            continue;

        // Cross-road actors have incompatible Frenet s coordinates;
        // they are handled by the plugin layer via world→Frenet projection.
        if (other.road_id != ego.road_id)
            continue;

        double ds = other.s - ego.s;
        double dt = other.t - ego.t;
        if (ego.lane_id > 0)
            ds = -ds;

        if (ds > lookahead || ds < -10.0)
            continue;
        if (std::fabs(dt) > std::max(lateralDist * 4.0, egoLaneWidth * 3.5))
            continue;

        double adjustedGap = ds - 0.5 * (ego.length + other.length);

        if (result.numObstacles < ScanResult::MAX_OBSTACLES)
        {
            Obstacle& obs   = result.obstacles[result.numObstacles];
            obs.objectIndex = static_cast<int>(i);
            obs.s           = other.s;
            obs.d           = other.t;
            obs.speed       = other.speed;
            obs.length      = other.length;
            obs.width       = other.width;
            ++result.numObstacles;
        }

        const int laneOffset = other.lane_id - ego.lane_id;

        LeadVehicleInfo info;
        info.exists      = true;
        info.objectIndex = static_cast<int>(i);
        info.gap         = adjustedGap;
        info.speed       = other.speed;
        info.lateralDist = dt;

        if (laneOffset == 0 && std::fabs(dt) < lateralDist)
        {
            if (adjustedGap > 0 && adjustedGap < result.surr.front.gap)
                result.surr.front = info;
        }
        else if (laneOffset > 0 || dt > egoLaneWidth * 0.3)
        {
            if (adjustedGap > 0 && adjustedGap < result.surr.leftFront.gap)
                result.surr.leftFront = info;
            else if (adjustedGap < 0 && std::fabs(adjustedGap) < std::fabs(result.surr.leftRear.gap))
            {
                LeadVehicleInfo rear = info;
                rear.gap             = std::fabs(adjustedGap);
                result.surr.leftRear = rear;
            }
        }
        else if (laneOffset < 0 || dt < -egoLaneWidth * 0.3)
        {
            if (adjustedGap > 0 && adjustedGap < result.surr.rightFront.gap)
                result.surr.rightFront = info;
            else if (adjustedGap < 0 && std::fabs(adjustedGap) < std::fabs(result.surr.rightRear.gap))
            {
                LeadVehicleInfo rear  = info;
                rear.gap              = std::fabs(adjustedGap);
                result.surr.rightRear = rear;
            }
        }
    }

    return result;
}

}  // namespace rsim_driver
