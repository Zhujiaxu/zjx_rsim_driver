/*
 * QpPathOptimizer — QP-based path smoothing using OSQP.
 *
 * Takes the coarse DP path and an (s, l) bound corridor (derived from
 * reference line limits and obstacle positions) and solves for a smooth
 * l(s) profile.
 */
#pragma once

#include "DriverTypes.hpp"
#include "DpPathOptimizer.hpp"
#include "IPredictor.hpp"
#include "ReferenceLine.hpp"

#include <vector>

namespace rsim_driver
{

class QpPathOptimizer
{
public:
    struct Config
    {
        double wRef       = 1.0;     // l tracking
        double wSmoothDdl = 1000.0;  // ddl smoothness
        double wSmoothDl  = 100.0;   // dl smoothness (optional)
        int    maxIter    = 4000;
    };

    Config config;

    // refLine: reference line (unused in current implementation — all computations in Frenet)
    // dpPath: coarse path from DpPathOptimizer (provides s_i positions)
    // lb, ub: lateral bounds at each s_i (lb ≤ l_i ≤ ub)
    bool Optimize(const ReferenceLine& /*refLine*/,
                  const std::vector<DpPathPoint>& dpPath,
                  const std::vector<double>& lb,
                  const std::vector<double>& ub,
                  std::vector<QpPathPoint>* qpPath) const;
};

}  // namespace rsim_driver
