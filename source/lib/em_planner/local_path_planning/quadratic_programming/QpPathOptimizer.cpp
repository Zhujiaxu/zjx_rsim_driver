#include "local_path_planning/quadratic_programming/QpPathOptimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace rsim_driver
{

namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kSolverTolerance = 1e-6;

int SIndex(int pointIndex)
{
    return 4 * pointIndex;
}

int LIndex(int pointIndex)
{
    return 4 * pointIndex + 1;
}

int LPrimeIndex(int pointIndex)
{
    return 4 * pointIndex + 2;
}

int LDoublePrimeIndex(int pointIndex)
{
    return 4 * pointIndex + 3;
}

bool IsFinite(double value)
{
    return std::isfinite(value);
}

double NonNegative(double value)
{
    return std::max(0.0, value);
}

bool ValidConfig(const QpPathOptimizerConfig& config)
{
    return IsFinite(config.weight_reference_l) &&
           IsFinite(config.weight_smooth_l_prime) &&
           IsFinite(config.weight_smooth_l_double_prime) &&
           IsFinite(config.weight_jerk) &&
           IsFinite(config.weight_collision) &&
           IsFinite(config.weight_drivable_area_center) &&
           IsFinite(config.collision_lateral_buffer) &&
           config.collision_lateral_buffer >= 0.0 &&
           config.max_iterations > 0;
}

bool ObstacleCoversS(const StaticFrenetObstacle& obstacle, double s)
{
    const double halfLength = 0.5 * std::max(0.0, obstacle.length);
    return s >= obstacle.s - halfLength && s <= obstacle.s + halfLength;
}

double ObstacleHalfExtent(const StaticFrenetObstacle& obstacle,
                          const QpPathOptimizerConfig& config)
{
    return 0.5 * std::max(std::max(0.0, obstacle.length),
                          std::max(0.0, obstacle.width)) +
           config.collision_lateral_buffer;
}

bool ValidInput(const CartesianFrenetState& start,
                const std::vector<DpPathPoint>& coarsePath,
                const DrivableArea& drivableArea)
{
    if (!cartesian_to_frenet_detail::IsFinite(start) ||
        coarsePath.size() < 2 ||
        drivableArea.left_boundary.size() != coarsePath.size() ||
        drivableArea.right_boundary.size() != coarsePath.size())
    {
        return false;
    }

    for (std::size_t i = 0; i < coarsePath.size(); ++i)
    {
        const DpPathPoint& point = coarsePath[i];
        const SlPoint& left = drivableArea.left_boundary[i];
        const SlPoint& right = drivableArea.right_boundary[i];
        if (!IsFinite(point.s) ||
            !IsFinite(point.l) ||
            !IsFinite(point.l_prime) ||
            !IsFinite(point.l_double_prime) ||
            !IsFinite(left.s) ||
            !IsFinite(left.l) ||
            !IsFinite(right.s) ||
            !IsFinite(right.l) ||
            right.l > left.l)
        {
            return false;
        }

        if (i > 0 && coarsePath[i].s - coarsePath[i - 1].s <= kEpsilon)
            return false;
    }

    return true;
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

void AddTargetCost(std::vector<Eigen::Triplet<double>>* triplets,
                   Eigen::VectorXd* gradient,
                   int index,
                   double target,
                   double weight)
{
    if (triplets == nullptr || gradient == nullptr || weight <= 0.0)
        return;

    triplets->emplace_back(index, index, 2.0 * weight);
    (*gradient)(index) += -2.0 * weight * target;
}

void AddConstraintRow(std::vector<Eigen::Triplet<double>>* triplets,
                      Eigen::VectorXd* lowerBound,
                      Eigen::VectorXd* upperBound,
                      int row,
                      const std::vector<std::pair<int, double>>& coeffs,
                      double lower,
                      double upper)
{
    for (const auto& coeff : coeffs)
        triplets->emplace_back(row, coeff.first, coeff.second);
    (*lowerBound)(row) = lower;
    (*upperBound)(row) = upper;
}

void AddCollisionCosts(const std::vector<DpPathPoint>& coarsePath,
                       const DrivableArea& drivableArea,
                       const std::vector<StaticFrenetObstacle>& staticObstacles,
                       const QpPathOptimizerConfig& config,
                       std::vector<Eigen::Triplet<double>>* hessianTriplets,
                       Eigen::VectorXd* gradient)
{
    const double weight = NonNegative(config.weight_collision);
    if (weight <= 0.0)
        return;

    for (std::size_t i = 0; i < coarsePath.size(); ++i)
    {
        const DpPathPoint& point = coarsePath[i];
        for (const StaticFrenetObstacle& obstacle : staticObstacles)
        {
            if (!IsFinite(obstacle.s) ||
                !IsFinite(obstacle.l) ||
                !IsFinite(obstacle.length) ||
                !IsFinite(obstacle.width) ||
                !ObstacleCoversS(obstacle, point.s))
            {
                continue;
            }

            const double halfExtent = ObstacleHalfExtent(obstacle, config);
            const double target =
                point.l >= obstacle.l
                    ? obstacle.l + halfExtent
                    : obstacle.l - halfExtent;
            const double clampedTarget =
                std::clamp(target,
                           drivableArea.right_boundary[i].l,
                           drivableArea.left_boundary[i].l);
            AddTargetCost(hessianTriplets,
                          gradient,
                          LIndex(static_cast<int>(i)),
                          clampedTarget,
                          weight);
        }
    }
}

double ComputeObjective(const std::vector<DpPathPoint>& path,
                        const DrivableArea& drivableArea,
                        const std::vector<StaticFrenetObstacle>& staticObstacles,
                        const QpPathOptimizerConfig& config)
{
    double objective = 0.0;
    const double wReference = NonNegative(config.weight_reference_l);
    const double wLPrime = NonNegative(config.weight_smooth_l_prime);
    const double wLDoublePrime = NonNegative(config.weight_smooth_l_double_prime);
    const double wJerk = NonNegative(config.weight_jerk);
    const double wCenter = NonNegative(config.weight_drivable_area_center);
    const double wCollision = NonNegative(config.weight_collision);

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        const DpPathPoint& point = path[i];
        const double center = 0.5 * (drivableArea.left_boundary[i].l +
                                     drivableArea.right_boundary[i].l);
        objective += wReference * point.l * point.l;
        objective += wLPrime * point.l_prime * point.l_prime;
        objective += wLDoublePrime * point.l_double_prime * point.l_double_prime;
        objective += wCenter * (point.l - center) * (point.l - center);

        if (wCollision <= 0.0)
            continue;

        for (const StaticFrenetObstacle& obstacle : staticObstacles)
        {
            if (!IsFinite(obstacle.s) ||
                !IsFinite(obstacle.l) ||
                !IsFinite(obstacle.length) ||
                !IsFinite(obstacle.width) ||
                !ObstacleCoversS(obstacle, point.s))
            {
                continue;
            }

            const double halfExtent = ObstacleHalfExtent(obstacle, config);
            const double target =
                point.l >= obstacle.l
                    ? obstacle.l + halfExtent
                    : obstacle.l - halfExtent;
            const double clampedTarget =
                std::clamp(target,
                           drivableArea.right_boundary[i].l,
                           drivableArea.left_boundary[i].l);
            objective += wCollision * (point.l - clampedTarget) *
                         (point.l - clampedTarget);
        }
    }

    for (std::size_t i = 0; i + 1 < path.size(); ++i)
    {
        const double ds = path[i + 1].s - path[i].s;
        if (ds <= kEpsilon)
            continue;
        const double jerkResidual =
            path[i + 1].l_double_prime + path[i].l_prime / ds;
        objective += wJerk * jerkResidual * jerkResidual;
    }

    return objective;
}

}  // namespace

QpPathOptimizer::QpPathOptimizer(const QpPathOptimizerConfig& config)
    : config_(config)
{
}

const QpPathOptimizerConfig& QpPathOptimizer::config() const
{
    return config_;
}

void QpPathOptimizer::SetConfig(const QpPathOptimizerConfig& config)
{
    config_ = config;
}

bool QpPathOptimizer::OptimizeFrenet(
    const CartesianFrenetState& start,
    const std::vector<DpPathPoint>& coarsePath,
    const DrivableArea& drivableArea,
    const std::vector<StaticFrenetObstacle>& staticObstacles,
    std::vector<DpPathPoint>* localfrenetpath,
    double* objective) const
{
    if (localfrenetpath == nullptr || objective == nullptr)
        return false;

    std::vector<DpPathPoint> output;
    double outputObjective = 0.0;
    if (!ValidConfig(config_) || !ValidInput(start, coarsePath, drivableArea))
    {
        *localfrenetpath = output;
        *objective = outputObjective;
        return false;
    }

    const int n = static_cast<int>(coarsePath.size());
    const int numVariables = 4 * n;
    const int numConstraints = n + 3 + n + 2 * (n - 1);

    std::vector<Eigen::Triplet<double>> hessianTriplets;
    hessianTriplets.reserve(static_cast<std::size_t>(numVariables * 6));
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numVariables);

    for (int i = 0; i < n; ++i)
    {
        AddTargetCost(&hessianTriplets,
                      &gradient,
                      LIndex(i),
                      0.0,
                      NonNegative(config_.weight_reference_l));
        AddTargetCost(&hessianTriplets,
                      &gradient,
                      LPrimeIndex(i),
                      0.0,
                      NonNegative(config_.weight_smooth_l_prime));
        AddTargetCost(&hessianTriplets,
                      &gradient,
                      LDoublePrimeIndex(i),
                      0.0,
                      NonNegative(config_.weight_smooth_l_double_prime));

        const double center =
            0.5 * (drivableArea.left_boundary[static_cast<std::size_t>(i)].l +
                   drivableArea.right_boundary[static_cast<std::size_t>(i)].l);
        AddTargetCost(&hessianTriplets,
                      &gradient,
                      LIndex(i),
                      center,
                      NonNegative(config_.weight_drivable_area_center));
    }

    const double wJerk = NonNegative(config_.weight_jerk);
    for (int i = 0; i + 1 < n; ++i)
    {
        const double ds = coarsePath[static_cast<std::size_t>(i + 1)].s -
                          coarsePath[static_cast<std::size_t>(i)].s;
        AddQuadraticTerm(&hessianTriplets,
                         {{LPrimeIndex(i), 1.0 / ds},
                          {LDoublePrimeIndex(i + 1), 1.0}},
                         wJerk);
    }

    AddCollisionCosts(coarsePath,
                      drivableArea,
                      staticObstacles,
                      config_,
                      &hessianTriplets,
                      &gradient);

    Eigen::SparseMatrix<double> hessian(numVariables, numVariables);
    hessian.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
    hessian.makeCompressed();

    std::vector<Eigen::Triplet<double>> constraintTriplets;
    constraintTriplets.reserve(static_cast<std::size_t>(numConstraints * 5));
    Eigen::VectorXd lowerBound(numConstraints);
    Eigen::VectorXd upperBound(numConstraints);

    int row = 0;
    for (int i = 0; i < n; ++i)
    {
        AddConstraintRow(&constraintTriplets,
                         &lowerBound,
                         &upperBound,
                         row++,
                         {{SIndex(i), 1.0}},
                         coarsePath[static_cast<std::size_t>(i)].s,
                         coarsePath[static_cast<std::size_t>(i)].s);
    }

    AddConstraintRow(&constraintTriplets,
                     &lowerBound,
                     &upperBound,
                     row++,
                     {{LIndex(0), 1.0}},
                     start.l,
                     start.l);
    AddConstraintRow(&constraintTriplets,
                     &lowerBound,
                     &upperBound,
                     row++,
                     {{LPrimeIndex(0), 1.0}},
                     start.l_prime,
                     start.l_prime);
    AddConstraintRow(&constraintTriplets,
                     &lowerBound,
                     &upperBound,
                     row++,
                     {{LDoublePrimeIndex(0), 1.0}},
                     start.l_double_prime,
                     start.l_double_prime);

    for (int i = 0; i < n; ++i)
    {
        const std::size_t index = static_cast<std::size_t>(i);
        double lowerL = drivableArea.right_boundary[index].l;
        double upperL = drivableArea.left_boundary[index].l;
        if (i == 0)
        {
            lowerL = std::min(lowerL, start.l);
            upperL = std::max(upperL, start.l);
        }
        AddConstraintRow(&constraintTriplets,
                         &lowerBound,
                         &upperBound,
                         row++,
                         {{LIndex(i), 1.0}},
                         lowerL,
                         upperL);
    }

    for (int i = 1; i < n; ++i)
    {
        const double delta = coarsePath[static_cast<std::size_t>(i)].s -
                             coarsePath[static_cast<std::size_t>(i - 1)].s;
        const double deltas = delta;
        const double deltas2 = deltas * deltas;
        const double deltas3 = deltas2 * deltas;
        const double previousLDoublePrimeCoeff = deltas3 / (6.0 * delta);
        const double currentLDoublePrimeCoeff =
            deltas2 / 2.0 - deltas3 / (6.0 * delta);

        AddConstraintRow(&constraintTriplets,
                         &lowerBound,
                         &upperBound,
                         row++,
                         {{LIndex(i), 1.0},
                          {LIndex(i - 1), -1.0},
                          {LPrimeIndex(i), -deltas},
                          {LDoublePrimeIndex(i - 1), previousLDoublePrimeCoeff},
                          {LDoublePrimeIndex(i), currentLDoublePrimeCoeff}},
                         0.0,
                         0.0);

        AddConstraintRow(&constraintTriplets,
                         &lowerBound,
                         &upperBound,
                         row++,
                         {{LPrimeIndex(i), 1.0},
                          {LPrimeIndex(i - 1), -1.0},
                          {LDoublePrimeIndex(i - 1), -0.5 * delta},
                          {LDoublePrimeIndex(i), -0.5 * delta}},
                         0.0,
                         0.0);
    }

    Eigen::SparseMatrix<double> linearMatrix(numConstraints, numVariables);
    linearMatrix.setFromTriplets(constraintTriplets.begin(),
                                 constraintTriplets.end());
    linearMatrix.makeCompressed();

    OsqpEigen::Solver solver;
    solver.settings()->setMaxIteration(config_.max_iterations);
    solver.settings()->setAbsoluteTolerance(kSolverTolerance);
    solver.settings()->setRelativeTolerance(kSolverTolerance);
    solver.settings()->setVerbosity(false);
    solver.settings()->setPolish(true);
    solver.settings()->setWarmStart(false);

    solver.data()->setNumberOfVariables(numVariables);
    solver.data()->setNumberOfConstraints(numConstraints);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setBounds(lowerBound, upperBound);

    if (!solver.initSolver())
    {
        *localfrenetpath = output;
        *objective = outputObjective;
        return false;
    }

    solver.solveProblem();
    const OsqpEigen::Status status = solver.getStatus();
    const bool ok = (status == OsqpEigen::Status::Solved ||
                     status == OsqpEigen::Status::SolvedInaccurate);
    if (!ok)
    {
        *localfrenetpath = output;
        *objective = outputObjective;
        return false;
    }

    const Eigen::VectorXd solution = solver.getSolution();
    output.reserve(coarsePath.size());
    for (int i = 0; i < n; ++i)
    {
        output.push_back({
            coarsePath[static_cast<std::size_t>(i)].s,
            solution(LIndex(i)),
            solution(LPrimeIndex(i)),
            solution(LDoublePrimeIndex(i)),
        });
    }

    outputObjective =
        ComputeObjective(output, drivableArea, staticObstacles, config_);
    *localfrenetpath = std::move(output);
    *objective = outputObjective;
    return true;
}

}  // namespace rsim_driver
