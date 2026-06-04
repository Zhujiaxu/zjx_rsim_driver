/*
 * ReferenceLineSmoother implementation.
 */
#include "ReferenceLineSmoother.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace rsim_driver
{
namespace
{

int XIndex(int pointIndex)
{
    return 2 * pointIndex;
}

int YIndex(int pointIndex)
{
    return 2 * pointIndex + 1;
}

void AddQuadraticTerm(std::vector<Eigen::Triplet<double>>* triplets,
                      const std::vector<std::pair<int, double>>& coeffs,
                      double weight)
{
    if (triplets == nullptr || weight <= 0.0)
        return;

    for (std::size_t i = 0; i < coeffs.size(); ++i)
    {
        for (std::size_t j = i; j < coeffs.size(); ++j)
        {
            int row = coeffs[i].first;
            int col = coeffs[j].first;
            if (row > col)
                std::swap(row, col);
            triplets->emplace_back(row,
                                   col,
                                   2.0 * weight * coeffs[i].second * coeffs[j].second);
        }
    }
}

}  // namespace

bool ReferenceLineSmoother::Smooth(const std::vector<ReferencePoint>& raw,
                                   std::vector<ReferencePoint>* smoothed) const
{
    if (smoothed == nullptr || raw.empty())
        return false;

    const int n = static_cast<int>(raw.size());
    if (n < 2)
    {
        *smoothed = raw;
        return true;
    }

    const int numVariables = 2 * n;
    std::vector<Eigen::Triplet<double>> hessianTriplets;
    hessianTriplets.reserve(static_cast<std::size_t>(numVariables * 8));

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numVariables);

    // Similarity: sum ||p_i - raw_i||^2.
    const double wSimilarity = std::max(0.0, config.similarityWeight);
    for (int i = 0; i < n; ++i)
    {
        AddQuadraticTerm(&hessianTriplets, {{XIndex(i), 1.0}}, wSimilarity);
        AddQuadraticTerm(&hessianTriplets, {{YIndex(i), 1.0}}, wSimilarity);
        gradient(XIndex(i)) += -2.0 * wSimilarity * raw[static_cast<std::size_t>(i)].x;
        gradient(YIndex(i)) += -2.0 * wSimilarity * raw[static_cast<std::size_t>(i)].y;
    }

    // Smoothness: every three points form p_{i-1} - 2p_i + p_{i+1}.
    const double wSmooth = std::max(0.0, config.smoothWeight);
    for (int i = 1; i < n - 1; ++i)
    {
        AddQuadraticTerm(&hessianTriplets,
                         {{XIndex(i - 1), 1.0}, {XIndex(i), -2.0}, {XIndex(i + 1), 1.0}},
                         wSmooth);
        AddQuadraticTerm(&hessianTriplets,
                         {{YIndex(i - 1), 1.0}, {YIndex(i), -2.0}, {YIndex(i + 1), 1.0}},
                         wSmooth);
    }

    // Compactness: every adjacent pair forms p_{i+1} - p_i.
    const double wCompact = std::max(0.0, config.compactWeight);
    for (int i = 0; i < n - 1; ++i)
    {
        AddQuadraticTerm(&hessianTriplets,
                         {{XIndex(i), -1.0}, {XIndex(i + 1), 1.0}},
                         wCompact);
        AddQuadraticTerm(&hessianTriplets,
                         {{YIndex(i), -1.0}, {YIndex(i + 1), 1.0}},
                         wCompact);
    }

    Eigen::SparseMatrix<double> hessian(numVariables, numVariables);
    hessian.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
    hessian.makeCompressed();

    std::vector<Eigen::Triplet<double>> constraintTriplets;
    constraintTriplets.reserve(static_cast<std::size_t>(numVariables));
    for (int i = 0; i < numVariables; ++i)
        constraintTriplets.emplace_back(i, i, 1.0);

    Eigen::SparseMatrix<double> linearMatrix(numVariables, numVariables);
    linearMatrix.setFromTriplets(constraintTriplets.begin(), constraintTriplets.end());
    linearMatrix.makeCompressed();

    const double bound = std::max(0.0, config.coordinateBound);
    Eigen::VectorXd lowerBound(numVariables);
    Eigen::VectorXd upperBound(numVariables);
    for (int i = 0; i < n; ++i)
    {
        const ReferencePoint& point = raw[static_cast<std::size_t>(i)];
        lowerBound(XIndex(i)) = point.x - bound;
        upperBound(XIndex(i)) = point.x + bound;
        lowerBound(YIndex(i)) = point.y - bound;
        upperBound(YIndex(i)) = point.y + bound;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setMaxIteration(config.maxIterations);
    solver.settings()->setVerbosity(false);
    solver.settings()->setPolish(false);
    solver.settings()->setWarmStart(false);

    solver.data()->setNumberOfVariables(numVariables);
    solver.data()->setNumberOfConstraints(numVariables);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setBounds(lowerBound, upperBound);

    if (!solver.initSolver())
        return false;

    solver.solveProblem();
    const OsqpEigen::Status status = solver.getStatus();
    const bool ok = (status == OsqpEigen::Status::Solved ||
                     status == OsqpEigen::Status::SolvedInaccurate);
    if (!ok)
        return false;

    const Eigen::VectorXd solution = solver.getSolution();
    *smoothed = raw;
    for (int i = 0; i < n; ++i)
    {
        ReferencePoint& point = (*smoothed)[static_cast<std::size_t>(i)];
        point.x = solution(XIndex(i));
        point.y = solution(YIndex(i));
    }

    return true;
}

}  // namespace rsim_driver
