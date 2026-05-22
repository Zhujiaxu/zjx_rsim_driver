/*
 * QpSpeedOptimizer — QP speed smoothing via OsqpEigen.
 */
#include "QpSpeedOptimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rsim_driver
{

bool QpSpeedOptimizer::Optimize(const FrenetState& ego,
                                const std::vector<DpSpeedPoint>& dpSpeed,
                                const std::vector<StBoundary>& stBoundaries,
                                double speedLimit,
                                double /*maxAccel*/,
                                double /*maxDecel*/,
                                std::vector<QpSpeedPoint>* qpSpeed) const
{
    if (!qpSpeed)
        return false;

    const int n = static_cast<int>(dpSpeed.size());
    if (n < 3)
        return false;

    const double dt = dpSpeed[1].t - dpSpeed[0].t;
    const double vRef = std::min(speedLimit, std::max(0.0, dpSpeed.back().v));

    // --- Build upper-triangular Hessian (P) as Eigen::SparseMatrix ---
    std::vector<Eigen::Triplet<double>> triplets;

    // wRefV: v_i = (s_{i+1} - s_{i-1}) / (2*dt)
    // v_i² contributes: [1/4dt²] * [s_{i-1}² - 2*s_{i-1}*s_{i+1} + s_{i+1}²]
    // QP: 0.5*xᵀPx → P entries = 2*w*(coefficients from f²)
    const double wV = config.wRefV / (4.0 * dt * dt);
    for (int i = 1; i < n - 1; ++i)
    {
        triplets.emplace_back(i - 1, i - 1,  2.0 * wV);
        triplets.emplace_back(i - 1, i + 1, -2.0 * wV);
        triplets.emplace_back(i + 1, i + 1,  2.0 * wV);
    }

    // wAccel: a_i = (s_{i+1} - 2s_i + s_{i-1}) / dt²
    const double wA = config.wAccel / (dt * dt * dt * dt);
    for (int i = 1; i < n - 1; ++i)
    {
        triplets.emplace_back(i - 1, i - 1,  2.0 * wA);
        triplets.emplace_back(i - 1, i,     -4.0 * wA);
        triplets.emplace_back(i - 1, i + 1,  2.0 * wA);
        triplets.emplace_back(i,     i,      8.0 * wA);
        triplets.emplace_back(i,     i + 1, -4.0 * wA);
        triplets.emplace_back(i + 1, i + 1,  2.0 * wA);
    }

    // wJerk: j_i = (s_{i+2} - 2s_{i+1} + 2s_{i-1} - s_{i-2}) / (2*dt³)
    const double wJ = config.wJerk / (4.0 * dt * dt * dt * dt * dt * dt);
    for (int i = 2; i < n - 2; ++i)
    {
        // Pattern: [1, -2, 0, 2, -1] at rows (i-2, i-1, i, i+1, i+2)
        // Upper triangle: row ≤ col
        triplets.emplace_back(i - 2, i - 2,  2.0 * wJ);
        triplets.emplace_back(i - 2, i - 1, -4.0 * wJ);
        triplets.emplace_back(i - 2, i + 1,  4.0 * wJ);
        triplets.emplace_back(i - 2, i + 2, -2.0 * wJ);
        triplets.emplace_back(i - 1, i - 1,  8.0 * wJ);
        triplets.emplace_back(i - 1, i + 1, -8.0 * wJ);
        triplets.emplace_back(i - 1, i + 2,  4.0 * wJ);
        triplets.emplace_back(i + 1, i + 1,  8.0 * wJ);
        triplets.emplace_back(i + 1, i + 2, -4.0 * wJ);
        triplets.emplace_back(i + 2, i + 2,  2.0 * wJ);
    }

    // Terminal costs
    const double wEv = config.wEndV / (dt * dt);  // v_{n-1} = (s_{n-1} - s_{n-2}) / dt
    triplets.emplace_back(n - 2, n - 2,  2.0 * wEv);
    triplets.emplace_back(n - 2, n - 1, -2.0 * wEv);
    triplets.emplace_back(n - 1, n - 1,  2.0 * wEv);
    triplets.emplace_back(n - 1, n - 1,  2.0 * config.wEndS);  // s_{n-1} tracking

    Eigen::SparseMatrix<double> hessian(n, n);
    hessian.setFromTriplets(triplets.begin(), triplets.end());

    // --- Gradient (q) ---
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);
    const double qV = -config.wRefV * vRef / dt;
    for (int i = 1; i < n - 1; ++i)
    {
        gradient(i - 1) += qV;   // coefficient for s_{i-1} in -2*vRef*v_i
        gradient(i + 1) -= qV;   // coefficient for s_{i+1} in -2*vRef*v_i
    }
    const double sRef = dpSpeed.back().s;
    gradient(n - 1) += -2.0 * config.wEndS * sRef;
    gradient(n - 2) -= -2.0 * config.wEndV * vRef / dt;
    gradient(n - 1) += -2.0 * config.wEndV * vRef / dt;

    // --- Linear constraints (A) = identity ---
    std::vector<Eigen::Triplet<double>> aTriplets;
    for (int i = 0; i < n; ++i)
        aTriplets.emplace_back(i, i, 1.0);
    Eigen::SparseMatrix<double> linearMatrix(n, n);
    linearMatrix.setFromTriplets(aTriplets.begin(), aTriplets.end());

    // --- Bounds ---
    Eigen::VectorXd lowerBound(n), upperBound(n);
    lowerBound(0) = ego.s - 1e-3;
    upperBound(0) = ego.s + 1e-3;

    for (int i = 1; i < n; ++i)
    {
        double t = dpSpeed[static_cast<size_t>(i)].t;
        double sLo = ego.s;
        double sHi = ego.s + speedLimit * t;

        for (const auto& b : stBoundaries)
        {
            if (std::fabs(b.t - t) < dt * 0.6)
            {
                if (dpSpeed[static_cast<size_t>(i)].s < b.sMin)
                    sHi = std::min(sHi, b.sMin);
                if (dpSpeed[static_cast<size_t>(i)].s > b.sMax)
                    sLo = std::max(sLo, b.sMax);
            }
        }

        sLo = std::max(sLo, ego.s);
        sHi = std::max(sHi, sLo + 0.1);
        lowerBound(i) = sLo;
        upperBound(i) = sHi;
    }

    // --- Build solver ---
    OsqpEigen::Solver solver;
    solver.settings()->setMaxIteration(config.maxIter);
    solver.settings()->setVerbosity(false);
    solver.settings()->setPolish(true);
    solver.settings()->setWarmStart(false);

    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(n);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setBounds(lowerBound, upperBound);

    if (!solver.initSolver())
        return false;

    OsqpEigen::ErrorExitFlag flag = solver.solveProblem();
    OsqpEigen::Status status = solver.getStatus();

    bool ok = (status == OsqpEigen::Status::Solved ||
               status == OsqpEigen::Status::SolvedInaccurate);
    if (!ok)
        return false;

    Eigen::VectorXd solution = solver.getSolution();

    // --- Build output ---
    qpSpeed->resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        (*qpSpeed)[static_cast<size_t>(i)].t = dpSpeed[static_cast<size_t>(i)].t;
        // OSQP can return solution(0) slightly outside the [ego.s-1e-3,
        // ego.s+1e-3] bound. Anchor s[0] explicitly so downstream consumers
        // (e.g. PathTimeTrajectory.sCoeffs[0]) match the ego state exactly.
        (*qpSpeed)[static_cast<size_t>(i)].s = (i == 0) ? ego.s : solution(i);

        if (i == 0)
        {
            (*qpSpeed)[static_cast<size_t>(i)].v = ego.s_d;
            (*qpSpeed)[static_cast<size_t>(i)].a = ego.s_dd;
        }
        else if (i == n - 1)
        {
            (*qpSpeed)[static_cast<size_t>(i)].v = (solution(i) - solution(i - 1)) / dt;
            (*qpSpeed)[static_cast<size_t>(i)].a = 0.0;
        }
        else
        {
            (*qpSpeed)[static_cast<size_t>(i)].v = (solution(i + 1) - solution(i - 1)) / (2.0 * dt);
            (*qpSpeed)[static_cast<size_t>(i)].a = (solution(i + 1) - 2.0 * solution(i) + solution(i - 1)) / (dt * dt);
        }
    }

    return true;
}

}  // namespace rsim_driver
