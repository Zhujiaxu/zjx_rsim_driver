/*
 * DpPathOptimizer implementation.
 */
#include "DpPathOptimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <vector>

namespace rsim_driver
{

bool DpPathOptimizer::Optimize(const ReferenceLine& refLine,
                               const FrenetState& ego,
                               const Obstacle* obstacles,
                               int numObstacles,
                               const IPredictor& predictor,
                               std::vector<DpPathPoint>* dpPath) const
{
    if (!dpPath)
        return false;

    const int nCols = std::max(2, config.numLongitudinal);
    const int nRows = std::max(3, config.numLateral);
    const double sStep = std::max(1.0, config.sStep);
    const double lStep = std::max(0.1, config.lStep);

    // lateral offsets centred on zero (lane centre)
    const double lMin = -lStep * static_cast<double>(nRows / 2);
    const double lMax =  lStep * static_cast<double>(nRows / 2);

    // longitudinal positions
    std::vector<double> sCols(nCols);
    for (int j = 0; j < nCols; ++j)
        sCols[j] = ego.s + sStep * static_cast<double>(j + 1);

    // lateral offsets
    std::vector<double> lRows(nRows);
    for (int i = 0; i < nRows; ++i)
        lRows[i] = lMin + lStep * static_cast<double>(i);

    // DP cost and predecessor tracking
    // cost[j][i] = min cost to reach node (j, i)
    // prev[j][i] = best lateral row index from column j-1
    std::vector<std::vector<double>> cost(nCols, std::vector<double>(nRows, DBL_MAX));
    std::vector<std::vector<int>> prev(nCols, std::vector<int>(nRows, -1));

    // Collision helper: check if (s, l) collides with any predicted obstacle
    auto collisionPenalty = [&](double s, double l, double t) -> double {
        if (obstacles == nullptr || numObstacles <= 0)
            return 0.0;

        for (int oi = 0; oi < numObstacles; ++oi)
        {
            const Obstacle& obs = obstacles[oi];
            double obsS = obs.s + obs.speed * t;
            double longDist = std::fabs(s - obsS);
            double longLimit = 0.5 * (5.0 + obs.length) + config.collisionMargin;
            if (longDist > longLimit)
                continue;

            double latDist = std::fabs(l - obs.d);
            double latLimit = 0.5 * (2.0 + obs.width) + config.collisionMargin;
            if (latDist < latLimit)
                return config.wCollision;
        }
        return 0.0;
    };

    // Column 0: cost = wRef * l_i² + collision
    double egoV = std::max(1.0, ego.s_d);
    for (int i = 0; i < nRows; ++i)
    {
        double t0 = (sCols[0] - ego.s) / egoV;
        cost[0][i] = config.wRefOffset * lRows[i] * lRows[i]
                   + collisionPenalty(sCols[0], lRows[i], t0);
    }

    // DP forward
    for (int j = 1; j < nCols; ++j)
    {
        double ds = sCols[j] - sCols[j - 1];
        double travelTime = (sCols[j] - ego.s) / egoV;

        for (int i = 0; i < nRows; ++i)
        {
            double bestC = DBL_MAX;
            int bestPrev = -1;

            for (int k = 0; k < nRows; ++k)
            {
                if (cost[j - 1][k] >= DBL_MAX * 0.5)
                    continue;

                double dl = (lRows[i] - lRows[k]) / ds;

                // ddl: curvature of the lateral profile
                double prevDl = 0.0;
                if (j >= 2 && prev[j - 1][k] >= 0)
                {
                    double dsPrev = sCols[j - 1] - sCols[j - 2];
                    int kk = prev[j - 1][k];
                    prevDl = (lRows[k] - lRows[kk]) / std::max(0.1, dsPrev);
                }
                double ddl = (dl - prevDl) / (0.5 * (ds + ds));

                double c = cost[j - 1][k]
                         + config.wRefOffset * lRows[i] * lRows[i]
                         + config.wSmooth * ddl * ddl
                         + collisionPenalty(sCols[j], lRows[i], travelTime);

                if (c < bestC)
                {
                    bestC = c;
                    bestPrev = k;
                }
            }

            cost[j][i] = bestC;
            prev[j][i] = bestPrev;
        }
    }

    // Find best at last column
    int bestLast = 0;
    double bestLastCost = DBL_MAX;
    for (int i = 0; i < nRows; ++i)
    {
        if (cost[nCols - 1][i] < bestLastCost)
        {
            bestLastCost = cost[nCols - 1][i];
            bestLast = i;
        }
    }
    if (bestLastCost >= DBL_MAX * 0.5)
        return false;

    // Trace back
    std::vector<int> bestRows(nCols);
    int cur = bestLast;
    for (int j = nCols - 1; j >= 0; --j)
    {
        bestRows[j] = cur;
        cur = prev[j][cur];
        if (j == 0)
            break;
    }

    // Build output path
    dpPath->resize(nCols);
    for (int j = 0; j < nCols; ++j)
    {
        (*dpPath)[j].s = sCols[j];
        (*dpPath)[j].l = lRows[bestRows[j]];

        if (j == 0)
        {
            (*dpPath)[j].dl  = (lRows[bestRows[j]] - ego.d) / (sCols[j] - ego.s);
            (*dpPath)[j].ddl = 0.0;
        }
        else
        {
            double ds = sCols[j] - sCols[j - 1];
            (*dpPath)[j].dl  = (lRows[bestRows[j]] - lRows[bestRows[j - 1]]) / ds;
            double prevDl = (j >= 2)
                ? ((*dpPath)[j - 1].l - lRows[bestRows[j - 2]]) / std::max(0.1, sCols[j - 1] - sCols[j - 2])
                : (lRows[bestRows[j - 1]] - ego.d) / (sCols[j - 1] - ego.s);
            (*dpPath)[j].ddl = ((*dpPath)[j].dl - prevDl) / (0.5 * (ds + sCols[j] - sCols[std::max(0, j - 2)]));
        }
    }

    return true;
}

}  // namespace rsim_driver
