#include "QpPathOptimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;
        constexpr double kSolverTolerance = 1e-6;
        constexpr double kInfinity = 1e12;

        int LIndex(int pointIndex)
        {
            return 3 * pointIndex;
        }

        int LPrimeIndex(int pointIndex)
        {
            return 3 * pointIndex + 1;
        }

        int LDoublePrimeIndex(int pointIndex)
        {
            return 3 * pointIndex + 2;
        }

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        double NonNegative(double value)
        {
            return std::max(0.0, value);
        }
        SlPoint InterpolateBoundary(const std::vector<SlPoint> &boundary, double s)
        {
            if (boundary.empty())
                return {s, 0.0};

            if (s <= boundary.front().s)
                return {s, boundary.front().l};

            if (s >= boundary.back().s)
                return {s, boundary.back().l};

            for (std::size_t i = 1; i < boundary.size(); ++i)
            {
                const SlPoint &prev = boundary[i - 1];
                const SlPoint &next = boundary[i];
                if (s <= next.s)
                {
                    const double ds = next.s - prev.s;
                    if (ds <= kEpsilon)
                        return {s, prev.l};
                    return {s, prev.l + (next.l - prev.l) * (s - prev.s) / ds};
                }
            }

            return {s, boundary.back().l};
        }

        bool ValidConfig(const QpPathOptimizerConfig &config)
        {
            return config.num_points >= 2 &&
                   IsFinite(config.ds) && config.ds > kEpsilon &&
                   IsFinite(config.ego_length) && config.ego_length > 0.0 &&
                   IsFinite(config.ego_width) && config.ego_width > 0.0 &&
                   IsFinite(config.weight_reference_l) &&
                   IsFinite(config.weight_smooth_l_prime) &&
                   IsFinite(config.weight_smooth_l_double_prime) &&
                   IsFinite(config.weight_jerk) &&
                   IsFinite(config.weight_drivable_area_center) &&
                   config.max_iterations > 0;
        }

        SlPoint InterpolateLeftBoundary(const std::vector<SlPoint> &boundary, double s)
        {
            if (boundary.empty())
                return {s, 0.0};

            if (s <= boundary.front().s)
                return {s, boundary.front().l};

            if (s >= boundary.back().s)
                return {s, boundary.back().l};

            for (std::size_t i = 1; i < boundary.size(); ++i)
            {
                const SlPoint &prev = boundary[i - 1];
                const SlPoint &next = boundary[i];
                if (s <= next.s)
                {
                    const double ds = next.s - prev.s;
                    if (ds <= kEpsilon)
                        return {s, prev.l};
                    return {s, std::min(prev.l, next.l)};
                }
            }

            return {s, boundary.back().l};
        }
        SlPoint InterpolateRightBoundary(const std::vector<SlPoint> &boundary, double s)
        {
            if (boundary.empty())
                return {s, 0.0};

            if (s <= boundary.front().s)
                return {s, boundary.front().l};

            if (s >= boundary.back().s)
                return {s, boundary.back().l};

            for (std::size_t i = 1; i < boundary.size(); ++i)
            {
                const SlPoint &prev = boundary[i - 1];
                const SlPoint &next = boundary[i];
                if (s <= next.s)
                {
                    const double ds = next.s - prev.s;
                    if (ds <= kEpsilon)
                        return {s, prev.l};
                    return {s, std::max(prev.l, next.l)};
                }
            }

            return {s, boundary.back().l};
        }

        bool ValidInput(const CartesianFrenetState &start,
                        const DrivableAreaResult &drivableArea)
        {
            if (!cartesian_to_frenet_detail::IsFinite(start))
                return false;

            if (drivableArea.left_boundary.size() < 2 ||
                drivableArea.left_boundary.size() != drivableArea.right_boundary.size())
            {
                return false;
            }

            for (std::size_t i = 0; i < drivableArea.left_boundary.size(); ++i)
            {
                const SlPoint &left = drivableArea.left_boundary[i];
                const SlPoint &right = drivableArea.right_boundary[i];
                if (!IsFinite(left.s) || !IsFinite(left.l) ||
                    !IsFinite(right.s) || !IsFinite(right.l) ||
                    std::fabs(left.s - right.s) > kEpsilon ||
                    right.l > left.l)
                {
                    return false;
                }
                if (i > 0 &&
                    (left.s - drivableArea.left_boundary[i - 1].s <= kEpsilon ||
                     right.s - drivableArea.right_boundary[i - 1].s <= kEpsilon))
                {
                    return false;
                }
            }

            return true;
        }

        void AddTargetCost(std::vector<Eigen::Triplet<double>> *triplets,
                           Eigen::VectorXd *gradient,
                           int index,
                           double target,
                           double weight)
        {
            if (triplets == nullptr || gradient == nullptr || weight <= 0.0)
                return;

            triplets->emplace_back(index, index, 2.0 * weight);
            (*gradient)(index) += -2.0 * weight * target;
        }

        void AddConstraintRow(std::vector<Eigen::Triplet<double>> *triplets,
                              Eigen::VectorXd *lowerBound,
                              Eigen::VectorXd *upperBound,
                              int row,
                              const std::vector<std::pair<int, double>> &coeffs,
                              double lower,
                              double upper)
        {
            for (const auto &coeff : coeffs)
                triplets->emplace_back(row, coeff.first, coeff.second);
            (*lowerBound)(row) = lower;
            (*upperBound)(row) = upper;
        }

    } // namespace

    QpPathOptimizer::QpPathOptimizer(const QpPathOptimizerConfig &config)
        : config_(config)
    {
    }

    const QpPathOptimizerConfig &QpPathOptimizer::config() const
    {
        return config_;
    }

    void QpPathOptimizer::SetConfig(const QpPathOptimizerConfig &config)
    {
        config_ = config;
    }

    bool QpPathOptimizer::Optimize(
        const CartesianFrenetState &start,
        const DrivableAreaResult &drivableArea,
        QpPathResult *result) const
    {

        if (result == nullptr)
            return false;

        *result = QpPathResult{};

        QpPathResult output;
        double outputObjective = 0.0;
        if (!ValidConfig(config_) || !ValidInput(start, drivableArea))
        {
            output.Flag = QpPathOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }

        const int n = config_.num_points;
        const double ds = config_.ds;
        const double halfLen = 0.5 * config_.ego_length;
        const double halfWid = 0.5 * config_.ego_width;
        const int numVariables = 3 * n;
        // 3 start eq + 2(n-1) Taylor eq + 2(n-1) two-sided corner rows.
        const int numConstraints = 4 * n - 1;

        // --- build uniform s grid ---
        std::vector<double> S(n);
        for (int i = 0; i < n; ++i)
            S[i] = start.s + static_cast<double>(i) * ds;

        // --- interpolate drivable area at uniform s and at corner positions ---
        // center_i = (left(s_i) + right(s_i)) / 2
        std::vector<double> centers(n);
        // corner boundaries
        std::vector<double> leftFront(n), rightFront(n);
        std::vector<double> leftRear(n), rightRear(n);
        for (int i = 0; i < n; ++i)
        {
            const double si = S[i];
            const SlPoint lb = InterpolateBoundary(drivableArea.left_boundary, si);
            const SlPoint rb = InterpolateBoundary(drivableArea.right_boundary, si);
            centers[i] = 0.5 * (lb.l + rb.l);

            const SlPoint lf = InterpolateLeftBoundary(drivableArea.left_boundary, si + halfLen);
            const SlPoint rf = InterpolateRightBoundary(drivableArea.right_boundary, si + halfLen);
            leftFront[i] = lf.l;
            rightFront[i] = rf.l;

            const SlPoint lr = InterpolateLeftBoundary(drivableArea.left_boundary, si - halfLen);
            const SlPoint rr = InterpolateRightBoundary(drivableArea.right_boundary, si - halfLen);
            leftRear[i] = lr.l;
            rightRear[i] = rr.l;
        }

        // --- early feasibility check for corner constraints (skip i=0) ---
        for (int i = 1; i < n; ++i)
        {
            if (rightFront[i] + halfWid > leftFront[i] - halfWid ||
                rightRear[i] + halfWid > leftRear[i] - halfWid)
            {
                output.Flag = QpPathOptimizerFallback::SolveFailStop;
                *result = std::move(output);
                return false;
            }
        }

        // ===== assemble cost (Hessian + gradient) =====
        std::vector<Eigen::Triplet<double>> hessianTriplets;
        hessianTriplets.reserve(static_cast<std::size_t>(numVariables * 4));
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numVariables);

        for (int i = 0; i < n; ++i)
        {
            // Reference line cost: w_ref * l_i^2
            AddTargetCost(&hessianTriplets, &gradient,
                          LIndex(i), 0.0,
                          NonNegative(config_.weight_reference_l));
            // Drivable area center cost: w_center * (l_i - c_i)^2
            AddTargetCost(&hessianTriplets, &gradient,
                          LIndex(i), centers[i],
                          NonNegative(config_.weight_drivable_area_center));
            // l' smoothness: w_l' * (l'_i)^2
            AddTargetCost(&hessianTriplets, &gradient,
                          LPrimeIndex(i), 0.0,
                          NonNegative(config_.weight_smooth_l_prime));
            // l'' smoothness: w_l'' * (l''_i)^2
            AddTargetCost(&hessianTriplets, &gradient,
                          LDoublePrimeIndex(i), 0.0,
                          NonNegative(config_.weight_smooth_l_double_prime));
        }

        // Jerk cost: (w_jerk / ds^2) * Σ (l''_{i+1} - l''_i)^2
        const double wJerk = NonNegative(config_.weight_jerk);
        if (wJerk > 0.0)
        {
            const double jerkScale = wJerk / (ds * ds);
            for (int i = 0; i + 1 < n; ++i)
            {
                // (l''_{i+1} - l''_i)^2 expands to:
                //   H[L''(i), L''(i)]     += 2 * jerkScale
                //   H[L''(i+1), L''(i+1)] += 2 * jerkScale
                //   H[L''(i), L''(i+1)]   -= 2 * jerkScale
                const double hVal = 2.0 * jerkScale;
                hessianTriplets.emplace_back(LDoublePrimeIndex(i),
                                             LDoublePrimeIndex(i), hVal);
                hessianTriplets.emplace_back(LDoublePrimeIndex(i + 1),
                                             LDoublePrimeIndex(i + 1), hVal);
                hessianTriplets.emplace_back(LDoublePrimeIndex(i),
                                             LDoublePrimeIndex(i + 1), -hVal);
            }
        }

        Eigen::SparseMatrix<double> hessian(numVariables, numVariables);
        hessian.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
        hessian.makeCompressed();

        // ===== assemble constraints =====
        std::vector<Eigen::Triplet<double>> constraintTriplets;
        constraintTriplets.reserve(static_cast<std::size_t>(numConstraints * 5));
        Eigen::VectorXd lowerBound(numConstraints);
        Eigen::VectorXd upperBound(numConstraints);

        int row = 0;

        // --- start state fixed (3 eq) ---
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{LIndex(0), 1.0}},
                         start.l, start.l);
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{LPrimeIndex(0), 1.0}},
                         start.l_prime, start.l_prime);
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{LDoublePrimeIndex(0), 1.0}},
                         start.l_double_prime, start.l_double_prime);

        // --- Taylor expansion (2(n-1) eq, i >= 1) ---
        // l_i - l_{i-1} - ds·l'_{i-1} - (ds²/3)·l''_{i-1} - (ds²/6)·l''_i = 0
        const double ds2Over3 = ds * ds / 3.0;
        const double ds2Over6 = ds * ds / 6.0;
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{LIndex(i), 1.0},
                              {LIndex(i - 1), -1.0},
                              {LPrimeIndex(i - 1), -ds},
                              {LDoublePrimeIndex(i - 1), -ds2Over3},
                              {LDoublePrimeIndex(i), -ds2Over6}},
                             0.0, 0.0);
        }

        // l'_i - l'_{i-1} - (ds/2) l''_{i-1} - (ds/2) l''_i = 0
        const double dsOver2 = ds / 2.0;
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{LPrimeIndex(i), 1.0},
                              {LPrimeIndex(i - 1), -1.0},
                              {LDoublePrimeIndex(i - 1), -dsOver2},
                              {LDoublePrimeIndex(i), -dsOver2}},
                             0.0, 0.0);
        }

        // --- corner constraints (2(n-1) two-sided rows, i >= 1) ---
        // Front: l_i + halfLen·l'_i ∈ [Right_f + halfWid, Left_f - halfWid]
        // Rear:  l_i - halfLen·l'_i ∈ [Right_r + halfWid, Left_r - halfWid]
        for (int i = 1; i < n; ++i)
        {
            // Front vehicle corner interval.
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{LIndex(i), 1.0},
                              {LPrimeIndex(i), halfLen}},
                             rightFront[i] + halfWid, leftFront[i] - halfWid);

            // Rear vehicle corner interval.
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{LIndex(i), 1.0},
                              {LPrimeIndex(i), -halfLen}},
                             rightRear[i] + halfWid, leftRear[i] - halfWid);
        }

        if (row != numConstraints)
        {
            output.Flag = QpPathOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }

        Eigen::SparseMatrix<double> linearMatrix(numConstraints, numVariables);
        linearMatrix.setFromTriplets(constraintTriplets.begin(),
                                     constraintTriplets.end());
        linearMatrix.makeCompressed();

        // ===== OSQP solve =====
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
            output.Flag = QpPathOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }

        solver.solveProblem();
        const OsqpEigen::Status status = solver.getStatus();
        const bool ok = (status == OsqpEigen::Status::Solved ||
                         status == OsqpEigen::Status::SolvedInaccurate);
        if (!ok)
        {
            output.Flag = QpPathOptimizerFallback::SolveFailStop;
            *result = std::move(output);
            return false;
        }

        // ===== extract solution =====
        const Eigen::VectorXd solution = solver.getSolution();
        if (solution.size() != numVariables || !solution.allFinite())
        {
            output.Flag = QpPathOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }
        output.localfrenetpath.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
        {
            output.localfrenetpath.push_back({
                S[i],
                solution(LIndex(i)),
                solution(LPrimeIndex(i)),
                solution(LDoublePrimeIndex(i)),
            });
        }

        // --- compute objective ---
        double obj = 0.0;
        const double wRef = NonNegative(config_.weight_reference_l);
        const double wCenter = NonNegative(config_.weight_drivable_area_center);
        const double wLP = NonNegative(config_.weight_smooth_l_prime);
        const double wLDP = NonNegative(config_.weight_smooth_l_double_prime);
        for (int i = 0; i < n; ++i)
        {
            const DpPathPoint &p = output.localfrenetpath[i];
            obj += wRef * p.l * p.l;
            obj += wCenter * (p.l - centers[i]) * (p.l - centers[i]);
            obj += wLP * p.l_prime * p.l_prime;
            obj += wLDP * p.l_double_prime * p.l_double_prime;
        }
        if (wJerk > 0.0)
        {
            for (int i = 0; i + 1 < n; ++i)
            {
                const DpPathPoint &p1 = output.localfrenetpath[i + 1];
                const DpPathPoint &p2 = output.localfrenetpath[i];
                const double diff = p1.l_double_prime - p2.l_double_prime;
                obj += (wJerk / (ds * ds)) * diff * diff;
            }
        }
        output.Flag = QpPathOptimizerFallback::Success;
        output.objective = obj;
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
