/*
 * TrafficRuleHandler — traffic-light query against the OpenDRIVE map.
 *
 * Q0: returns "no traffic light" stub. Q1 will plumb MapHelper for real OpenDriveParser
 * lookahead; signal phase map is supplied by the caller (本期可空).
 */

#pragma once

#include "DriverTypes.hpp"
#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <map>
#include <string>

namespace rsim_driver
{
    using rsim_plugin::ActorState;

    // Forward decl — Q1 brings in the real type.
    class MapHelper;

    struct TrafficLightInfo
    {
        bool        found          = false;
        int         signalId       = -1;
        double      distToStopLine = LARGE_NUMBER;
        std::string state;  // "red", "yellow", "green", ""
    };

    class TrafficRuleHandler
    {
    public:
        // Look ahead of `ego` along the OpenDRIVE map for the closest stop-line signal,
        // resolve its dependent traffic light, and report the cached phase from `tlStates`.
        TrafficLightInfo CheckTrafficLight(const ActorState&                  ego,
                                            const MapHelper*                   map,
                                            const std::map<int, std::string>& tlStates,
                                            double                             lookaheadDist) const;
    };

}  // namespace rsim_driver
