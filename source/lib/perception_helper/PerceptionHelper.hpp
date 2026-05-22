/*
 * PerceptionHelper — multi-lane surrounding vehicle detection.
 * Scans plugin SDK ActorState[] and classifies into front / leftFront / leftRear / rightFront / rightRear.
 *
 * Cross-road scanning (where ego and other are on different OpenDRIVE roads) is left to Q1
 * once MapHelper provides road connectivity. The Q0 implementation handles same-road case
 * (which already covers all the basic cruise / car-follow / cutin examples on a single road).
 */

#pragma once

#include "DriverTypes.hpp"
#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <vector>

namespace rsim_driver
{
    using rsim_plugin::ActorState;

    struct ScanResult
    {
        SurroundingVehicles surr;

        static constexpr int MAX_OBSTACLES = 32;
        Obstacle obstacles[MAX_OBSTACLES];
        int      numObstacles = 0;
    };

    class PerceptionHelper
    {
    public:
        // Scan all entities and classify surrounding vehicles relative to ego.
        //   ego         - controlled actor (caller passes by const ref).
        //   actors      - full TickContext.actors list (ego may or may not be included; same-id rows are skipped).
        //   lateralDist - max lateral distance for same-lane detection (m).
        //   lookahead   - max longitudinal scan range (m).
        //   egoLaneWidth - lane width at ego pos (m). Q0 callers can pass 3.5 fallback;
        //                  Q1 will plumb MapHelper-provided width.
        ScanResult Scan(const ActorState&              ego,
                        const std::vector<ActorState>& actors,
                        double                         lateralDist,
                        double                         lookahead,
                        double                         egoLaneWidth = 3.5) const;
    };

}  // namespace rsim_driver
