#pragma once

#include "MapHelper.hpp"

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace rsim_driver
{

struct GlobalPathPoint
{
    double x = 0.0;
    double y = 0.0;
};

struct GlobalPathRouteSegment
{
    int64_t road_id = 0;
    int lane_id = 0;
    double s_start = 0.0;
    double s_end = 0.0;
    double s_sign = 1.0;
    double t = 0.0;
    std::size_t wp_start_idx = 0;
    std::size_t wp_end_idx = 0;
};

struct GlobalPathResult
{
    std::vector<GlobalPathRouteSegment> route_segments;
    std::vector<GlobalPathPoint> world_points;
    double chord_length = 0.0;
};

class GlobalPathGenerator
{
public:
    bool GenerateFromXosc(const std::string& xoscPath,
                          const std::string& entityName,
                          const MapHelper& map,
                          GlobalPathResult* result) const;
    bool WriteCsv(const std::string& path,
                  const std::vector<GlobalPathPoint>& points) const;
};

double ComputeChordLength(const std::vector<GlobalPathPoint>& points);
bool WriteGlobalPathCsv(const std::string& path,
                        const std::vector<GlobalPathPoint>& points);

}  // namespace rsim_driver
