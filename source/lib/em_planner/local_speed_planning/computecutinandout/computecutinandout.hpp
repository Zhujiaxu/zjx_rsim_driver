#pragma once

#include "localpathreferline.hpp"
#include "perception/FrenetObstaclePerception.hpp"

#include <cstdint>
#include <vector>

namespace rsim_driver
{

struct ComputeCutInAndOutConfig
{
    double half_vehicle_width_l = 1.0;
    double ldot_epsilon = 1e-6;
};

struct CutInAndOutInfo
{
    int32_t id = 0;
    double tin = 0.0;
    double tout = 0.0;
    double sin = 0.0;
    double sout = 0.0;
};

class ComputeCutInAndOut
{
public:
    explicit ComputeCutInAndOut(const ComputeCutInAndOutConfig& config = {});

    const ComputeCutInAndOutConfig& config() const;
    void SetConfig(const ComputeCutInAndOutConfig& config);

    bool Compute(const localreferencelinepath& referenceLine,
                 const DynamicFrenetObstaclePerceptionResult& obstacles,
                 std::vector<CutInAndOutInfo>* result) const;

private:
    ComputeCutInAndOutConfig config_;
};

}  // namespace rsim_driver
