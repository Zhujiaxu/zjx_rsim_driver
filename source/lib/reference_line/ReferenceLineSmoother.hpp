/*
 * ReferenceLineSmoother — constrained QP smoothing for reference-line points.
 */
#pragma once

#include "ReferenceLine.hpp"

#include <vector>

namespace rsim_driver
{

class ReferenceLineSmoother
{
public:
    struct Config
    {
        double smoothWeight     = 100.0;  // second-difference cost
        double similarityWeight = 10.0;   // stay close to raw path
        double compactWeight    = 1.0;    // adjacent-point compactness
        double coordinateBound  = 0.1;    // raw x/y +/- bound (m)
        int    maxIterations    = 4000;
    };

    Config config;

    bool Smooth(const std::vector<ReferencePoint>& raw,
                std::vector<ReferencePoint>* smoothed) const;

    
};

}  // namespace rsim_driver
