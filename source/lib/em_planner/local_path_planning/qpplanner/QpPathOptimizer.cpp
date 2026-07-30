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
        constexpr double kSolverTolerance = 1e-5;
        constexpr double kInfinity = 1e12;
        constexpr int kEnvelopeSamplesPerInterval = 2;

        struct EnvelopeConstraintSample
        {
            int interval_index = 0;
            double ratio = 0.0;
            double s = 0.0;
            double lower_l = 0.0;
            double upper_l = 0.0;
        };

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

        double MinimumLeftBoundaryInRange(
            const std::vector<SlPoint> &boundary,
            double beginS,
            double endS)
        {
            double minimum = std::min(
                InterpolateLeftBoundary(boundary, beginS).l,
                InterpolateLeftBoundary(boundary, endS).l);
            for (const SlPoint &point : boundary)
            {
                if (point.s >= beginS - kEpsilon &&
                    point.s <= endS + kEpsilon)
                {
                    minimum = std::min(minimum, point.l);
                }
            }
            return minimum;
        }

        double MaximumRightBoundaryInRange(
            const std::vector<SlPoint> &boundary,
            double beginS,
            double endS)
        {
            double maximum = std::max(
                InterpolateRightBoundary(boundary, beginS).l,
                InterpolateRightBoundary(boundary, endS).l);
            for (const SlPoint &point : boundary)
            {
                if (point.s >= beginS - kEpsilon &&
                    point.s <= endS + kEpsilon)
                {
                    maximum = std::max(maximum, point.l);
                }
            }
            return maximum;
        }

        bool ValidInput(const StartPointFrenetState &start,
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

        std::vector<double> BuildSValues(const StartPointFrenetState &start,
                                         const QpPathOptimizerConfig &config,
                                         double maxS,
                                         double *step)
        {
            std::vector<double> values;
            if (step == nullptr || std::isnan(maxS))
                return values;

            const int maxIntervals = config.num_points - 1;
            const double nominalEnd =
                start.s + static_cast<double>(maxIntervals) * config.ds;
            const double endS = std::isfinite(maxS)
                                    ? std::min(nominalEnd, maxS)
                                    : nominalEnd;
            const double span = endS - start.s;
            if (!std::isfinite(endS) || span <= kEpsilon)
                return values;

            const int intervalCount = std::min(
                maxIntervals,
                std::max(1, static_cast<int>(std::ceil(span / config.ds))));
            *step = span / static_cast<double>(intervalCount);
            values.reserve(static_cast<std::size_t>(intervalCount) + 1U);
            for (int i = 0; i <= intervalCount; ++i)
                values.push_back(start.s + static_cast<double>(i) * *step);
            values.back() = endS;
            return values;
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
        const StartPointFrenetState &start,
        const DrivableAreaResult &drivableArea,
        QpPathResult *result,
        double max_s) const
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

        double ds = 0.0;
        const std::vector<double> S = BuildSValues(start, config_, max_s, &ds);
        if (S.size() < 2U)
        {
            PluginLogEcho("【QpPathOptimizer::Optimize】: SL 过短的s\n");
            output.Flag = QpPathOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }
        const int n = static_cast<int>(S.size());
        const double halfLen = 0.5 * config_.ego_length;
        const double halfWid = 0.5 * config_.ego_width;
        const int intervalCount = n - 1;
        const int numVariables = 3 * n;
        const int envelopeConstraintCount =
            intervalCount * kEnvelopeSamplesPerInterval;
        // 3 start eq + 2(n-1) Taylor eq + two envelope samples per interval.
        const int numConstraints =
            3 + 2 * intervalCount + envelopeConstraintCount;

        // Use the full axis-aligned vehicle envelope for knot targets.
        std::vector<double> centers(n);
        for (int i = 0; i < n; ++i)
        {
            const double si = S[i];
            const double left = MinimumLeftBoundaryInRange(
                drivableArea.left_boundary, si - halfLen, si + halfLen);
            const double right = MaximumRightBoundaryInRange(
                drivableArea.right_boundary, si - halfLen, si + halfLen);
            centers[i] = 0.5 * (left + right);
        }

        // Sample each QP interval at its midpoint and endpoint. Each sample
        // constrains the continuous constant-jerk interpolation used by the Taylor rows,
        // instead of forcing a discontinuous obstacle boundary onto a knot.
        std::vector<EnvelopeConstraintSample> envelopeSamples;
        envelopeSamples.reserve(
            static_cast<std::size_t>(envelopeConstraintCount));
        for (int interval = 0; interval < intervalCount; ++interval)
        {
            for (int sample = 1;
                 sample <= kEnvelopeSamplesPerInterval;
                 ++sample)
            {
                const double ratio =
                    static_cast<double>(sample) /
                    static_cast<double>(kEnvelopeSamplesPerInterval);
                const double sampleS = S[interval] + ratio * ds;
                const double left = MinimumLeftBoundaryInRange(
                    drivableArea.left_boundary,
                    sampleS - halfLen,
                    sampleS + halfLen);
                const double right = MaximumRightBoundaryInRange(
                    drivableArea.right_boundary,
                    sampleS - halfLen,
                    sampleS + halfLen);
                const double lower = right + halfWid;
                const double upper = left - halfWid;
                if (lower > upper)
                {
                    PluginLogEcho(
                        "【QpPathOptimizer】:包络可行域为空 interval=%d "
                        "ratio=%.2f s=%.6f lower=%.6f upper=%.6f\n",
                        interval, ratio, sampleS, lower, upper);
                    output.Flag = QpPathOptimizerFallback::SolveFailStop;
                    *result = std::move(output);
                    return false;
                }
                envelopeSamples.push_back(
                    {interval, ratio, sampleS, lower, upper});
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

        // --- continuous axis-aligned SL envelope constraints ---
        for (const EnvelopeConstraintSample &sample : envelopeSamples)
        {
            const int i = sample.interval_index;
            if (sample.ratio >= 1.0 - kEpsilon)
            {
                AddConstraintRow(
                    &constraintTriplets, &lowerBound, &upperBound, row++,
                    {{LIndex(i + 1), 1.0}},
                    sample.lower_l, sample.upper_l);
                continue;
            }

            const double u = sample.ratio;
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double accelCurrentCoeff =
                ds * ds * (0.5 * u2 - u3 / 6.0);
            const double accelNextCoeff = ds * ds * u3 / 6.0;
            AddConstraintRow(
                &constraintTriplets, &lowerBound, &upperBound, row++,
                {{LIndex(i), 1.0},
                 {LPrimeIndex(i), ds * u},
                 {LDoublePrimeIndex(i), accelCurrentCoeff},
                 {LDoublePrimeIndex(i + 1), accelNextCoeff}},
                sample.lower_l, sample.upper_l);
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
            PluginLogEcho(
                "【QpPathOptimizer】:OSQP求解失败 status=%d points=%d "
                "start_s=%.6f start_l=%.6f\n",
                static_cast<int>(status), n, start.s, start.l);
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
