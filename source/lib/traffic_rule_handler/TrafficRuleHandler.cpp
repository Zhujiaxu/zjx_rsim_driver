/*
 * TrafficRuleHandler — implementation.
 * Scans signals on ego's current road, follows dependences to the controlling traffic light,
 * and reports the cached phase from `tlStates`.
 *
 * `tlStates` is owned by the caller (the plugin main). Signal-phase wiring (xosc
 * <TrafficSignalControllerAction> tracking) is out of scope this iteration; any caller passing
 * an empty map will simply see `found=false`.
 */

#include "TrafficRuleHandler.hpp"
#include "MapHelper.hpp"

#include "RoadManager.hpp"

#include <vector>

namespace rsim_driver
{

TrafficLightInfo TrafficRuleHandler::CheckTrafficLight(const ActorState&                  ego,
                                                       const MapHelper*                   map,
                                                       const std::map<int, std::string>& tlStates,
                                                       double                             lookaheadDist) const
{
    TrafficLightInfo result;
    if (map == nullptr)
        return result;
    auto* odr = map->odr();
    if (odr == nullptr)
        return result;

    auto* road = odr->GetRoadById(ego.road_id);
    if (road == nullptr)
        return result;

    for (int i = 0; i < road->GetNumberOfSignals(); ++i)
    {
        roadmanager::Signal* sig = road->GetSignal(i);
        if (sig == nullptr)
            continue;

        if (sig->GetName() != "InvisibleStopLine")
            continue;

        const double signalS = sig->GetS();
        const double dist    = signalS - ego.s;

        if (dist < 0 || dist > lookaheadDist)
            continue;
        if (dist >= result.distToStopLine)
            continue;

        std::vector<int> deps = sig->GetDependences();
        for (int depId : deps)
        {
            roadmanager::Signal* tlSignal = odr->GetSignalById(depId);
            if (tlSignal == nullptr)
                continue;

            auto it = tlStates.find(tlSignal->GetId());
            if (it != tlStates.end())
            {
                result.found          = true;
                result.signalId       = tlSignal->GetId();
                result.distToStopLine = dist;
                result.state          = it->second;
                break;
            }
        }
    }

    return result;
}

}  // namespace rsim_driver
