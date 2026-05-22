/*
 * MapHelper — thin wrapper around OpenDriveParser for the driver plugin.
 *
 * Centralises all road-network queries the plugin needs:
 *   - Load(xodr_path)         load OpenDRIVE map (process-global singleton inside OpenDriveParser).
 *   - GetLaneWidth(road,s,lane)
 *   - LaneToWorld(road,lane,s,offset) -> (x,y,z,h)
 *   - GetLeftLane / GetRightLane neighbour id (0 if none).
 *   - DistToJunction ahead.
 *   - odr() raw pointer escape hatch (used by TrafficRuleHandler for signal lookup).
 *
 * Anything more elaborate (full route computation, Frenet refS arc-length) belongs in Q2's
 * route-construction step.
 */

#pragma once

#include <cstdint>
#include <string>

namespace roadmanager
{
    class OpenDrive;
}

namespace rsim_driver
{
    struct WorldPose
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double h = 0.0;
        bool   valid = false;
    };

    class MapHelper
    {
    public:
        // Loads the xodr into the OpenDriveParser process-global instance.
        // Returns false if the file can't be loaded.
        bool Load(const std::string& xodr_path);

        bool IsLoaded() const { return loaded_; }

        // Lane width at the given (road, s, lane). Returns 3.5 if the road or lane cannot be resolved.
        double GetLaneWidth(int64_t road_id, double s, int lane_id) const;

        // World-frame pose for an OpenDRIVE (road, lane, s) plus a lateral offset (m).
        WorldPose LaneToWorld(int64_t road_id, int lane_id, double s, double lateralOffset) const;

        // Neighbour lane ids. 0 = no neighbour on that side.
        int GetLeftLane(int64_t road_id, double s, int lane_id) const;
        int GetRightLane(int64_t road_id, double s, int lane_id) const;

        // LARGE_NUMBER if no junction ahead on this road.
        double DistToJunctionAhead(int64_t road_id, double s) const;

        // Total length of an OpenDRIVE road in metres. Returns 0.0 if unknown.
        double GetRoadLength(int64_t road_id) const;

        // Resolve the lane id corresponding to a track-frame (road_id, s, t) point.
        // Returns 0 if the position cannot be resolved.
        int TrackTToLane(int64_t road_id, double s, double t) const;

        // Convert a track-frame (road, s, t) directly to world pose, bypassing
        // lane-centre semantics.  Useful when the caller already knows the
        // lateral offset from the road reference line (e.g. from xosc).
        WorldPose TrackToWorld(int64_t road_id, double s, double t) const;

        // Raw access for code that still needs OpenDriveParser internals (e.g. TrafficRuleHandler).
        roadmanager::OpenDrive* odr() const;

    private:
        bool loaded_ = false;
    };

}  // namespace rsim_driver
