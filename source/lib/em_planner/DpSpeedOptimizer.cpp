/*
 * DpSpeedOptimizer implementation.
 */
#include "DpSpeedOptimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <vector>

namespace rsim_driver
{

bool DpSpeedOptimizer::Optimize(const FrenetState& ego,
                                double desiredSpeed,
                                double speedLimit,
                                const std::vector<StBoundary>& stBoundaries,
                                double horizonTime,
                                std::vector<DpSpeedPoint>* dpSpeed) const
{
    if (!dpSpeed)
        return false;

    const double dt  = std::max(0.2, config.tStep);
    const double ds  = std::max(0.5, config.sStep);
    const double vRef = std::max(0.0, std::min(desiredSpeed, speedLimit));

    const int nSteps = std::max(2, static_cast<int>(std::ceil(horizonTime / dt)));
    const double actualHorizon = dt * static_cast<double>(nSteps);
    const double sMin0 = ego.s;

    // Pre-compute max s at each time step
    std::vector<double> tGrid(nSteps + 1);
    std::vector<double> sMaxGrid(nSteps + 1);
    for (int j = 0; j <= nSteps; ++j)
    {
        tGrid[j]     = dt * static_cast<double>(j);
        sMaxGrid[j]  = sMin0 + speedLimit * tGrid[j];
    }

    // collision penalty at (t, s)
    auto collisionPenalty = [&](double t, double s) -> double {
        for (const auto& b : stBoundaries)
        {
            if (std::fabs(b.t - t) < dt * 0.6 &&
                s >= b.sMin && s <= b.sMax)
                return config.wCollision;
        }
        return 0.0;
    };

    // DP: for each time step, store node (s value, cost, prev s, prev v)
    struct Node
    {
        double s    = 0.0;
        double cost = DBL_MAX;
        double prevS = 0.0;
        double prevV = 0.0;
    };

    std::vector<std::vector<Node>> dp(nSteps + 1);

    // Step 0: initial state
    {
        int nRows0 = 1;
        dp[0].resize(1);
        dp[0][0].s    = sMin0;
        dp[0][0].cost = 0.0;
    }

    // Forward DP
    for (int j = 1; j <= nSteps; ++j)
    {
        double sMinJ = sMin0;                              // worst case: stopped
        double sMaxJ = sMaxGrid[j];                         // max speed
        int nRows    = std::max(2, static_cast<int>((sMaxJ - sMinJ) / ds) + 1);

        dp[j].resize(nRows);
        for (int i = 0; i < nRows; ++i)
            dp[j][i].s = sMinJ + ds * static_cast<double>(i);

        for (int i = 0; i < nRows; ++i)
        {
            double si = dp[j][i].s;
            double bestCost = DBL_MAX;
            double bestPrevS = sMin0;
            double bestPrevV = 0.0;

            for (size_t k = 0; k < dp[j - 1].size(); ++k)
            {
                if (dp[j - 1][k].cost >= DBL_MAX * 0.5)
                    continue;

                double sk = dp[j - 1][k].s;
                double v  = (si - sk) / dt;

                if (v < -1e-3 || v > speedLimit + 1e-3)
                    continue;

                double prevV = dp[j - 1][k].prevV;
                double a     = (v - prevV) / dt;

                double c = dp[j - 1][k].cost
                         + config.wRefV * (v - vRef) * (v - vRef)
                         + config.wAccel * a * a
                         + collisionPenalty(tGrid[j], si);

                if (c < bestCost)
                {
                    bestCost  = c;
                    bestPrevS = sk;
                    bestPrevV = v;
                }
            }

            dp[j][i].cost  = bestCost;
            dp[j][i].prevS = bestPrevS;
            dp[j][i].prevV = bestPrevV;
        }
    }

    // Find best terminal state
    int bestI = 0;
    double bestCost = DBL_MAX;
    for (int i = 0; i < static_cast<int>(dp[nSteps].size()); ++i)
    {
        if (dp[nSteps][i].cost < bestCost)
        {
            bestCost = dp[nSteps][i].cost;
            bestI    = i;
        }
    }

    if (bestCost >= DBL_MAX * 0.5)
        return false;

    // Trace forward: reconstruct path from step 0 to step nSteps
    std::vector<double> sPath;
    {
        int curI = bestI;
        double curS = dp[nSteps][curI].s;
        sPath.resize(nSteps + 1);
        sPath[nSteps] = curS;

        for (int j = nSteps; j >= 1; --j)
        {
            double prevS = dp[j][curI].prevS;
            // find the index in dp[j-1] that matches prevS
            for (size_t k = 0; k < dp[j - 1].size(); ++k)
            {
                if (std::fabs(dp[j - 1][k].s - prevS) < ds * 0.6)
                {
                    curS = dp[j - 1][k].s;
                    sPath[j - 1] = curS;
                    curI = static_cast<int>(k);
                    break;
                }
            }
        }
    }

    // Build output
    dpSpeed->resize(nSteps + 1);
    for (int j = 0; j <= nSteps; ++j)
    {
        (*dpSpeed)[j].t = tGrid[j];
        (*dpSpeed)[j].s = sPath[j];

        if (j == 0)
        {
            (*dpSpeed)[j].v = ego.s_d;
            (*dpSpeed)[j].a = ego.s_dd;
        }
        else
        {
            double v = (sPath[j] - sPath[j - 1]) / dt;
            (*dpSpeed)[j].v = v;
            (*dpSpeed)[j].a = (j >= 2) ? (v - (*dpSpeed)[j - 1].v) / dt : 0.0;
        }
    }

    return true;
}

}  // namespace rsim_driver
