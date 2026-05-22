/*
 * QpPathOptimizer — QP path smoothing via OsqpEigen.
 */
#include "QpPathOptimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace rsim_driver
{

bool QpPathOptimizer::Optimize(const ReferenceLine& /*refLine*/,
                               const std::vector<DpPathPoint>& dpPath,
                               const std::vector<double>& lb,
                               const std::vector<double>& ub,
                               std::vector<QpPathPoint>* qpPath) const
{
    if (!qpPath)
        return false;

    const int n = static_cast<int>(dpPath.size());
    if (n < 2)
        return false;

    const double ds = dpPath[1].s - dpPath[0].s;

    // --- Build upper-triangular Hessian (P) as Eigen::SparseMatrix ---
    std::vector<Eigen::Triplet<double>> triplets;

    // Diagonal from wRef: cost = wRef * sum(l_i²), QP: 0.5*xᵀPx → P[i][i] = 2*wRef
    for (int i = 0; i < n; ++i)
        triplets.emplace_back(i, i, 2.0 * config.wRef);

    // ddl smoothness: ddl_i = (l_{i+1} - 2l_i + l_{i-1}) / ds²
    // In QP: coefficient of f(x)² is w, and 0.5*xᵀPx → P = 2*w * (coefficient matrix of f²)
    const double wS = config.wSmoothDdl / (ds * ds * ds * ds);
    for (int i = 1; i < n - 1; ++i)
    {
        triplets.emplace_back(i - 1, i - 1,  2.0 * wS);
        triplets.emplace_back(i - 1, i,     -4.0 * wS);
        triplets.emplace_back(i - 1, i + 1,  2.0 * wS);
        triplets.emplace_back(i,     i,      8.0 * wS);
        triplets.emplace_back(i,     i + 1, -4.0 * wS);
        triplets.emplace_back(i + 1, i + 1,  2.0 * wS);
    }

    // dl smoothness: dl_i = (l_{i+1} - l_{i-1}) / (2*ds)
    const double wDl = config.wSmoothDl / (4.0 * ds * ds);
    for (int i = 1; i < n - 1; ++i)
    {
        triplets.emplace_back(i - 1, i - 1,  2.0 * wDl);
        triplets.emplace_back(i - 1, i + 1, -2.0 * wDl);
        triplets.emplace_back(i + 1, i + 1,  2.0 * wDl);
    }

    Eigen::SparseMatrix<double> hessian(n, n);
    hessian.setFromTriplets(triplets.begin(), triplets.end());

    // --- Gradient (q) — zero for path optimisation ---
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);

    // --- Linear constraints (A) = identity, one bound per variable ---
    std::vector<Eigen::Triplet<double>> aTriplets;
    for (int i = 0; i < n; ++i)
        aTriplets.emplace_back(i, i, 1.0);
    Eigen::SparseMatrix<double> linearMatrix(n, n);
    linearMatrix.setFromTriplets(aTriplets.begin(), aTriplets.end());

    // --- Bounds — kept alive for the solver lifetime ---
    Eigen::VectorXd lowerBound(n), upperBound(n);
    for (int i = 0; i < n; ++i)
    {
        lowerBound(i) = lb[static_cast<size_t>(i)];
        upperBound(i) = ub[static_cast<size_t>(i)];
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

    // --- Build output path ---
    qpPath->resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        (*qpPath)[static_cast<size_t>(i)].s = dpPath[static_cast<size_t>(i)].s;
        (*qpPath)[static_cast<size_t>(i)].l = solution(i);

        if (i == 0 && n > 1)
        {
            (*qpPath)[static_cast<size_t>(i)].dl  = (solution(1) - solution(0)) / ds;
            (*qpPath)[static_cast<size_t>(i)].ddl = 0.0;
        }
        else if (i == n - 1 && n > 1)
        {
            (*qpPath)[static_cast<size_t>(i)].dl  = (solution(n - 1) - solution(n - 2)) / ds;
            (*qpPath)[static_cast<size_t>(i)].ddl = 0.0;
        }
        else
        {
            (*qpPath)[static_cast<size_t>(i)].dl  = (solution(i + 1) - solution(i - 1)) / (2.0 * ds);
            (*qpPath)[static_cast<size_t>(i)].ddl = (solution(i + 1) - 2.0 * solution(i) + solution(i - 1)) / (ds * ds);
        }
    }

    return true;
}

}  // namespace rsim_driver
