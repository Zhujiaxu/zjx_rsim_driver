#include "GlobalPathGenerator.hpp"

#include "pugixml.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace rsim_driver
{

namespace
{

struct XoscRoutePoint
{
    bool has_road = false;
    int64_t road_id = 0;
    int lane_id = 0;
    double s = 0.0;
    double t = 0.0;
    GlobalPathPoint world;
};

bool NodeNameIs(const pugi::xml_node& node, const char* name)
{
    return std::strcmp(node.name(), name) == 0;
}

pugi::xml_node FindFirstDescendant(const pugi::xml_node& node, const char* name)
{
    for (pugi::xml_node child : node.children())
    {
        if (NodeNameIs(child, name))
            return child;
        pugi::xml_node nested = FindFirstDescendant(child, name);
        if (nested)
            return nested;
    }
    return {};
}

bool AttrDouble(const pugi::xml_node& node, const char* name, double* out)
{
    pugi::xml_attribute attr = node.attribute(name);
    if (!attr)
        return false;
    try
    {
        *out = std::stod(attr.value());
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool AttrInt64(const pugi::xml_node& node, const char* name, int64_t* out)
{
    pugi::xml_attribute attr = node.attribute(name);
    if (!attr)
        return false;
    try
    {
        *out = std::stoll(attr.value());
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParseXoscRoadPosition(const pugi::xml_node& roadPos,
                           const MapHelper& map,
                           XoscRoutePoint* out)
{
    int64_t road_id = 0;
    double s = 0.0;
    double t = 0.0;
    if (!AttrInt64(roadPos, "roadId", &road_id) ||
        !AttrDouble(roadPos, "s", &s))
    {
        return false;
    }
    AttrDouble(roadPos, "t", &t);

    const WorldPose pose = map.TrackToWorld(road_id, s, t);
    if (!pose.valid)
        return false;

    out->has_road = true;
    out->road_id = road_id;
    out->s = s;
    out->t = t;
    out->lane_id = map.TrackTToLane(road_id, s, t);
    out->world.x = pose.x;
    out->world.y = pose.y;
    return true;
}

bool ParseXoscWorldPosition(const pugi::xml_node& worldPos,
                            XoscRoutePoint* out)
{
    double x = 0.0;
    double y = 0.0;
    if (!AttrDouble(worldPos, "x", &x) ||
        !AttrDouble(worldPos, "y", &y))
    {
        return false;
    }
    out->has_road = false;
    out->world.x = x;
    out->world.y = y;
    return true;
}

bool IsDuplicateXoscPoint(const XoscRoutePoint& a, const XoscRoutePoint& b)
{
    const double dx = a.world.x - b.world.x;
    const double dy = a.world.y - b.world.y;
    if (dx * dx + dy * dy > 1e-6)
        return false;
    if (a.has_road != b.has_road)
        return false;
    if (!a.has_road)
        return true;
    return a.road_id == b.road_id &&
           std::fabs(a.s - b.s) < 1e-6 &&
           std::fabs(a.t - b.t) < 1e-6;
}

bool ParseXoscPolyline(const pugi::xml_node& polyline,
                       const MapHelper& map,
                       std::vector<XoscRoutePoint>* out)
{
    std::vector<XoscRoutePoint> parsed;
    for (pugi::xml_node vertex : polyline.children("Vertex"))
    {
        pugi::xml_node position = vertex.child("Position");
        if (!position)
            continue;

        XoscRoutePoint point;
        if (pugi::xml_node roadPos = position.child("RoadPosition"))
        {
            if (!ParseXoscRoadPosition(roadPos, map, &point))
                continue;
        }
        else if (pugi::xml_node worldPos = position.child("WorldPosition"))
        {
            if (!ParseXoscWorldPosition(worldPos, &point))
                continue;
        }
        else
        {
            continue;
        }

        if (!parsed.empty() && IsDuplicateXoscPoint(parsed.back(), point))
            continue;
        parsed.push_back(point);
    }

    if (parsed.size() < 4)
        return false;

    *out = std::move(parsed);
    return true;
}

bool TryParseFollowTrajectoryActions(const pugi::xml_node& node,
                                     const MapHelper& map,
                                     std::vector<XoscRoutePoint>* out)
{
    if (NodeNameIs(node, "FollowTrajectoryAction"))
    {
        pugi::xml_node polyline = FindFirstDescendant(node, "Polyline");
        if (polyline && ParseXoscPolyline(polyline, map, out))
            return true;
    }

    for (pugi::xml_node child : node.children())
    {
        if (TryParseFollowTrajectoryActions(child, map, out))
            return true;
    }
    return false;
}

bool TryExtractXoscTrajectory(const pugi::xml_node& node,
                              const std::string& entityName,
                              const MapHelper& map,
                              std::vector<XoscRoutePoint>* out)
{
    if (NodeNameIs(node, "Private"))
    {
        pugi::xml_attribute entityRef = node.attribute("entityRef");
        if (entityRef && entityName == entityRef.value() &&
            TryParseFollowTrajectoryActions(node, map, out))
        {
            return true;
        }
    }

    for (pugi::xml_node child : node.children())
    {
        if (TryExtractXoscTrajectory(child, entityName, map, out))
            return true;
    }
    return false;
}

std::vector<GlobalPathRouteSegment> BuildRouteSegmentsFromXoscPoints(
    const std::vector<XoscRoutePoint>& points,
    const MapHelper& map)
{
    std::vector<GlobalPathRouteSegment> routeSegments;
    std::size_t i = 0;
    while (i < points.size())
    {
        if (!points[i].has_road)
        {
            ++i;
            continue;
        }

        const int64_t road_id = points[i].road_id;
        std::size_t j = i;
        int first_nonzero_lane = 0;
        while (j < points.size() && points[j].has_road &&
               points[j].road_id == road_id)
        {
            if (first_nonzero_lane == 0 && points[j].lane_id != 0)
                first_nonzero_lane = points[j].lane_id;
            ++j;
        }

        GlobalPathRouteSegment segment;
        segment.road_id = road_id;
        segment.s_start = points[i].s;
        segment.s_end = points[j - 1].s;
        segment.s_sign = (segment.s_end >= segment.s_start) ? 1.0 : -1.0;
        segment.t = points[i].t;
        segment.lane_id = (first_nonzero_lane != 0)
                              ? first_nonzero_lane
                              : map.TrackTToLane(segment.road_id,
                                                 segment.s_start,
                                                 segment.t);
        segment.wp_start_idx = i;
        segment.wp_end_idx = j;
        routeSegments.push_back(segment);
        i = j;
    }
    return routeSegments;
}

}  // namespace

bool GlobalPathGenerator::GenerateFromXosc(const std::string& xoscPath,
                                           const std::string& entityName,
                                           const MapHelper& map,
                                           GlobalPathResult* result) const
{
    if (result == nullptr)
        return false;
    result->route_segments.clear();
    result->world_points.clear();
    result->chord_length = 0.0;

    if (xoscPath.empty())
        return false;

    pugi::xml_document doc;
    const pugi::xml_parse_result load = doc.load_file(xoscPath.c_str());
    if (!load)
    {
        std::fprintf(stderr,
                     "[GlobalPathGenerator] WARNING: routeXoscPath parse failed '%s': %s\n",
                     xoscPath.c_str(), load.description());
        return false;
    }

    std::vector<XoscRoutePoint> points;
    if (!TryExtractXoscTrajectory(doc, entityName, map, &points))
    {
        std::fprintf(stderr,
                     "[GlobalPathGenerator] WARNING: no valid FollowTrajectoryAction for entityRef=\"%s\"\n",
                     entityName.c_str());
        return false;
    }

    result->route_segments = BuildRouteSegmentsFromXoscPoints(points, map);
    result->world_points.reserve(points.size());
    for (const auto& point : points)
        result->world_points.push_back(point.world);

    if (result->route_segments.empty() || result->world_points.size() < 4)
    {
        result->route_segments.clear();
        result->world_points.clear();
        std::fprintf(stderr,
                     "[GlobalPathGenerator] WARNING: XOSC trajectory route has insufficient points/segments\n");
        return false;
    }

    result->chord_length = ComputeChordLength(result->world_points);
    return true;
}

bool GlobalPathGenerator::WriteCsv(
    const std::string& path,
    const std::vector<GlobalPathPoint>& points) const
{
    return WriteGlobalPathCsv(path, points);
}

double ComputeChordLength(const std::vector<GlobalPathPoint>& points)
{
    if (points.size() < 2)
        return 0.0;
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double dx = points[i].x - points[i - 1].x;
        const double dy = points[i].y - points[i - 1].y;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

bool WriteGlobalPathCsv(const std::string& path,
                        const std::vector<GlobalPathPoint>& points)
{
    if (path.empty() || points.empty())
        return true;

    std::FILE* fp = std::fopen(path.c_str(), "w");
    if (fp == nullptr)
    {
        std::fprintf(stderr,
                     "[GlobalPathGenerator] WARNING: cannot write global path CSV: %s\n",
                     path.c_str());
        return false;
    }

    std::fprintf(fp, "index,arc_s,delta_s,x,y\n");
    double arcS = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        double ds = 0.0;
        if (i > 0)
        {
            const double dx = points[i].x - points[i - 1].x;
            const double dy = points[i].y - points[i - 1].y;
            ds = std::sqrt(dx * dx + dy * dy);
            arcS += ds;
        }

        const auto& point = points[i];
        std::fprintf(fp, "%zu,%.9f,%.9f,%.9f,%.9f\n",
                     i, arcS, ds, point.x, point.y);
    }
    std::fclose(fp);
    return true;
}

}  // namespace rsim_driver
