#pragma once

#include "localpathreferline.hpp"
#include "FrenetObstaclePerception.hpp"

#include <cstdint>
#include <vector>

namespace rsim_driver
{

struct ComputeCutInAndOutConfig
{
    double half_vehicle_width_l = 1.0;
    double ldot_epsilon = 1e-6;
    double stationary_overlap_horizon = 8.0;
};

struct CutInAndOutInfo
{
    int32_t id = 0;
    double tin = 0.0;
    double tout = 0.0;
    double sin = 0.0;
    double sinmin=0.0;
    double sinmax=0.0;
    double sout = 0.0;
    double soutmin=0.0;
    double soutmax=0.0;
};

class ComputeCutInAndOut
{
public:
    explicit ComputeCutInAndOut(const ComputeCutInAndOutConfig& config = {});

    const ComputeCutInAndOutConfig& config() const;
    void SetConfig(const ComputeCutInAndOutConfig& config);

    bool Compute(const localreferencelinepath& referenceLine,
                 const DynamicFrenetObstaclePerceptionResult& obstacles,
                 double ego_s_dot,
                 double planningPeriod,
                 double t_plan,
                 std::vector<CutInAndOutInfo>* result,
                 std::vector<VirtualObstacleSeed>* seeds) const;

private:
    ComputeCutInAndOutConfig config_;
};

}  // namespace rsim_driver
