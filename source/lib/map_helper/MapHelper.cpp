/*
 * MapHelper — implementation.
 * Wraps OpenDriveParser singleton (roadmanager::Position::GetOpenDrive()).
 */

#include "MapHelper.hpp"
#include "RoadManager.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

namespace
{
constexpr double kLargeNumber = 1e30;

roadmanager::OpenDrive* GlobalOdr()
{
    return roadmanager::Position::GetOpenDrive();
}
}  // namespace

bool MapHelper::Load(const std::string& xodr_path)
{
    auto* odr = GlobalOdr();
    if (!odr)
    {
        loaded_ = false;
        return false;
    }
    loaded_ = odr->LoadOpenDriveFile(xodr_path.c_str(), /*replace=*/true);
    return loaded_;
}

roadmanager::OpenDrive* MapHelper::odr() const
{
    return GlobalOdr();
}

double MapHelper::GetLaneWidth(int64_t road_id, double s, int lane_id) const
{
    auto* odr = GlobalOdr();
    if (!odr)
        return 3.5;
    auto* road = odr->GetRoadById(road_id);
    if (!road)
        return 3.5;
    double w = road->GetLaneWidthByS(s, lane_id);
    return (w > 0.1) ? w : 3.5;
}

WorldPose MapHelper::LaneToWorld(int64_t road_id, int lane_id, double s, double lateralOffset) const
{
    WorldPose out;
    auto* odr = GlobalOdr();
    if (!odr)
        return out;

    roadmanager::Position pos;
    if (pos.SetLanePos(road_id, lane_id, s, lateralOffset) != roadmanager::Position::ReturnCode::OK)
        return out;
    out.x     = pos.GetX();
    out.y     = pos.GetY();
    out.z     = pos.GetZ();
    out.h     = pos.GetH();
    out.valid = true;
    return out;
}

int MapHelper::GetLeftLane(int64_t road_id, double s, int lane_id) const
{
    // Convention (OpenDRIVE): "left" relative to driving direction means lane_id moves toward 0
    // for negative ids and away from 0 for positive ids. Practically we step lane_id by ±1
    // and check the neighbour exists in the LaneSection.
    auto* odr = GlobalOdr();
    if (!odr || lane_id == 0)
        return 0;
    auto* road = odr->GetRoadById(road_id);
    if (!road)
        return 0;
    auto* section = road->GetLaneSectionByS(s);
    if (!section)
        return 0;

    int candidate = (lane_id < 0) ? lane_id + 1 : lane_id + 1;
    if (candidate == 0)
        candidate = (lane_id < 0) ? 1 : 0;  // skip across reference line for negative side
    if (!section->GetLaneById(candidate))
        return 0;
    return candidate;
}

int MapHelper::GetRightLane(int64_t road_id, double s, int lane_id) const
{
    auto* odr = GlobalOdr();
    if (!odr || lane_id == 0)
        return 0;
    auto* road = odr->GetRoadById(road_id);
    if (!road)
        return 0;
    auto* section = road->GetLaneSectionByS(s);
    if (!section)
        return 0;

    int candidate = (lane_id < 0) ? lane_id - 1 : lane_id - 1;
    if (candidate == 0)
        candidate = (lane_id < 0) ? 0 : -1;
    if (!section->GetLaneById(candidate))
        return 0;
    return candidate;
}

double MapHelper::DistToJunctionAhead(int64_t road_id, double s) const
{
    auto* odr = GlobalOdr();
    if (!odr)
        return kLargeNumber;
    auto* road = odr->GetRoadById(road_id);
    if (!road)
        return kLargeNumber;
    // OpenDRIVE: a road belonging to a junction has junction_id >= 0; otherwise -1.
    // Without a multi-road graph traversal we can only report whether *this* road is in a junction.
    // Q2 will refine using route information.
    if (road->GetJunction() >= 0)
        return std::max(0.0, 0.0 - 0.0);  // already on junction
    (void) s;
    return kLargeNumber;
}

double MapHelper::GetRoadLength(int64_t road_id) const
{
    auto* odr = GlobalOdr();
    if (!odr)
        return 0.0;
    auto* road = odr->GetRoadById(road_id);
    return road ? road->GetLength() : 0.0;
}

int MapHelper::TrackTToLane(int64_t road_id, double s, double t) const
{
    auto* odr = GlobalOdr();
    if (!odr)
        return 0;
    roadmanager::Position pos;
    if (pos.SetTrackPos(road_id, s, t) != roadmanager::Position::ReturnCode::OK)
        return 0;
    return pos.GetLaneId();
}

WorldPose MapHelper::TrackToWorld(int64_t road_id, double s, double t) const
{
    WorldPose out;
    auto* odr = GlobalOdr();
    if (!odr)
        return out;
    roadmanager::Position pos;
    if (pos.SetTrackPos(road_id, s, t) != roadmanager::Position::ReturnCode::OK)
        return out;
    out.x     = pos.GetX();
    out.y     = pos.GetY();
    out.z     = pos.GetZ();
    out.h     = pos.GetH();
    out.valid = true;
    return out;
}

}  // namespace rsim_driver
