#pragma once

#include "localpathreferline.hpp"
#include "FrenetObstaclePerception.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rsim_driver
{

struct ComputeCutInAndOutConfig
{
    double half_buffers = 1.5;
    double ldot_epsilon = 1e-6;
    double stationary_overlap_horizon = 8.0;
};

struct CutInAndOutInfo
{
    int32_t id = -1;
    double tin = -1.0;
    double tout = -1.0;
    double sin = -1.0;
    double sinmin=-1.0;
    double sinmax=-1.0;
    double sout = -1.0;
    double soutmin=-1.0;
    double soutmax=-1.0;
};

class ComputeCutInAndOut
{
public:
    explicit ComputeCutInAndOut(const ComputeCutInAndOutConfig& config = {});

    const ComputeCutInAndOutConfig& config() const;
    void SetConfig(const ComputeCutInAndOutConfig& config);

    bool Compute(const std::vector<SpeedReferenceLinePoint>& referenceLine,
                 const DynamicFrenetObstaclePerceptionResult& obstacles,
                 const double & ego_speed,
                 std::vector<CutInAndOutInfo>* result) const;

private:
    ComputeCutInAndOutConfig config_;
    mutable std::unordered_map<int32_t, int> slow_lead_counter_;
    mutable std::unordered_map<int32_t, int> oncoming_conflict_counter_;
};

}  // namespace rsim_driver
