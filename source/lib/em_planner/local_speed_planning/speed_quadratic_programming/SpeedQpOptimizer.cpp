#include "SpeedQpOptimizer.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;
        constexpr double kSolverTolerance = 1e-6;

        int SIndex(int i)
        {
            return 3 * i;
        }

        int VIndex(int i)
        {
            return 3 * i + 1;
        }

        int AIndex(int i)
        {
            return 3 * i + 2;
        }

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        double NonNegative(double value)
        {
            return std::max(0.0, value);
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

        bool ValidConfig(const QpSpeedOptimizerConfig &config)
        {
            return IsFinite(config.dt) && config.dt > kEpsilon &&
                   IsFinite(config.dqpt) && config.dqpt > kEpsilon &&
                   IsFinite(config.reference_speed) && config.reference_speed >= 0.0 &&
                   IsFinite(config.ego_length) && config.ego_length > 0.0 &&
                   IsFinite(config.longitudinal_safety_buffer) && config.longitudinal_safety_buffer >= 0.0 &&
                   IsFinite(config.weight_reference_speed) &&
                   IsFinite(config.weight_acceleration) &&
                   IsFinite(config.weight_jerk) &&
                   IsFinite(config.weight_progress) &&
                   IsFinite(config.a_min) &&
                   IsFinite(config.a_max) && config.a_min <= config.a_max &&
                   config.max_iterations > 0;
        }

        bool ValidInput(const DynamicPlanSpeedPoint &start,
                        const StDrivableAreaResult &drivable_area,
                        const int &expected_num_points)
        {
            if (!IsFinite(start.s) || !IsFinite(start.v) || !IsFinite(start.a))
                return false;
            if (start.v < -kEpsilon)
                return false;

            const std::size_t n = static_cast<std::size_t>(expected_num_points);
            if (drivable_area.lower_boundary.size() < n ||
                drivable_area.upper_boundary.size() < n)
                return false;
            return true;
        }

    } // namespace

    SpeedQpOptimizer::SpeedQpOptimizer(const QpSpeedOptimizerConfig &config)
        : config_(config)
    {
    }

    const QpSpeedOptimizerConfig &SpeedQpOptimizer::config() const
    {
        return config_;
    }

    void SpeedQpOptimizer::SetConfig(const QpSpeedOptimizerConfig &config)
    {
        config_ = config;
    }

    bool SpeedQpOptimizer::Optimize(const DynamicPlanSpeedPoint &start,
                                    const StDrivableAreaResult &drivable_area,
                                    QpSpeedOptimizerResult *result) const
    {
        if (result == nullptr)
            return false;
        const double halfEgo = 0.5 * config_.ego_length;
        const double totalMargin = halfEgo + config_.longitudinal_safety_buffer;
        const int n = std::floor(drivable_area.upper_boundary.back().t / config_.dt)+1;
        QpSpeedOptimizerResult output;

        if (!ValidConfig(config_) ||
            !ValidInput(start, drivable_area, n))
        {
            *result = output;
            return false;
        }

        const double dt = config_.dt;
        const int numVariables = 3 * n;
        const int numConstraints = 6 * n - 2;

        // ===== assemble cost =====
        std::vector<Eigen::Triplet<double>> hessianTriplets;
        hessianTriplets.reserve(static_cast<std::size_t>(4 * n));
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numVariables);

        // reference speed cost: w_ref * Σ (v_i - v_ref)^2
        const double wRef = NonNegative(config_.weight_reference_speed);
        if (wRef > 0.0)
        {
            for (int i = 0; i < n; ++i)
            {
                AddTargetCost(&hessianTriplets, &gradient,
                              VIndex(i), config_.reference_speed, wRef);
            }
        }

        // acceleration cost: w_acc * Σ a_i^2
        const double wAcc = NonNegative(config_.weight_acceleration);
        if (wAcc > 0.0)
        {
            for (int i = 0; i < n; ++i)
            {
                AddTargetCost(&hessianTriplets, &gradient,
                              AIndex(i), 0.0, wAcc);
            }
        }

        // progress cost: w_progress * (s_{N-1} - s_target)^2
        const double wProgress = NonNegative(config_.weight_progress);
        if (wProgress > 0.0)
        {
            AddTargetCost(&hessianTriplets, &gradient,
                          SIndex(n - 1), drivable_area.upper_boundary.back().s - totalMargin, wProgress);
        }

        // jerk cost: (w_jerk / dt^2) * Σ (a_{i+1} - a_i)^2
        const double wJerk = NonNegative(config_.weight_jerk);
        if (wJerk > 0.0)
        {
            const double jerkScale = wJerk / (dt * dt);
            for (int i = 0; i + 1 < n; ++i)
            {
                const double hVal = 2.0 * jerkScale;
                hessianTriplets.emplace_back(AIndex(i), AIndex(i), hVal);
                hessianTriplets.emplace_back(AIndex(i + 1), AIndex(i + 1), hVal);
                hessianTriplets.emplace_back(AIndex(i), AIndex(i + 1), -hVal);
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

        // --- start state equality (3) ---
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{SIndex(0), 1.0}},
                         start.s, start.s);
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{VIndex(0), 1.0}},
                         start.v, start.v);
        AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                         {{AIndex(0), 1.0}},
                         start.a, start.a);

        // --- s 3rd-order Taylor: s_{i+1} - s_i - dt*v_i - dt²/3*a_i - dt²/6*a_{i+1} = 0 ---
        const double dt2Over3 = dt * dt / 3.0;
        const double dt2Over6 = dt * dt / 6.0;
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{SIndex(i), 1.0},
                              {SIndex(i - 1), -1.0},
                              {VIndex(i - 1), -dt},
                              {AIndex(i - 1), -dt2Over3},
                              {AIndex(i), -dt2Over6}},
                             0.0, 0.0);
        }

        // --- v trapezoidal: v_{i+1} - v_i - dt/2*a_i - dt/2*a_{i+1} = 0 ---
        const double dtOver2 = dt / 2.0;
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{VIndex(i), 1.0},
                              {VIndex(i - 1), -1.0},
                              {AIndex(i - 1), -dtOver2},
                              {AIndex(i), -dtOver2}},
                             0.0, 0.0);
        }

        // --- ST drivable area with vehicle length (n-1 box constraints, skip i=0: fixed by start) ---
        // Apply ego half-length + buffer only to bounds that represent obstacle constraints
        // (not to open-road bounds at 0 or reference_line_total_length)

        for (int i = 1; i < n; ++i)
        {
            const double j = i * config_.dt / config_.dqpt;

            // Calculate the lower and upper bounds for the drivable area
            const double lb = drivable_area.lower_boundary[static_cast<std::size_t>(j)].s + totalMargin;
            const double ub = drivable_area.upper_boundary[static_cast<std::size_t>(j)].s - totalMargin;

            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{SIndex(i), 1.0}},
                             lb, ub);
        }

        // --- acceleration bounds (n box constraints) ---
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{AIndex(i), 1.0}},
                             config_.a_min, config_.a_max);
        }

        // --- no-reverse velocity bounds: v_i >= 0 (n rows) ---
        const double infinity = OsqpEigen::INFTY;
        for (int i = 0; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{VIndex(i), 1.0}},
                             0.0, infinity);
        }

        // --- no-reverse progress: s_i - s_{i-1} >= 0 (n-1 rows) ---
        for (int i = 1; i < n; ++i)
        {
            AddConstraintRow(&constraintTriplets, &lowerBound, &upperBound, row++,
                             {{SIndex(i), 1.0},
                              {SIndex(i - 1), -1.0}},
                             0.0, infinity);
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
            output.Flag = QpSpeedOptimizerFallback::Other;
            *result = std::move(output);
            return false;
        }

        solver.solveProblem();
        const OsqpEigen::Status status = solver.getStatus();
        const bool ok = (status == OsqpEigen::Status::Solved ||
                         status == OsqpEigen::Status::SolvedInaccurate);
        if (!ok)
        {
            output.Flag = QpSpeedOptimizerFallback::Stop;
            *result = std::move(output);
            return false;
        }

        const Eigen::VectorXd solution = solver.getSolution();

        // ===== extract solution =====
        output.stpoints.reserve(static_cast<std::size_t>(n + 1));
        for (int i = 0; i < n; ++i)
        {
            DynamicPlanSpeedPoint pt;
            pt.t = static_cast<double>(i) * dt;
            pt.s = solution(SIndex(i));
            pt.v = solution(VIndex(i));
            pt.a = solution(AIndex(i));
            output.stpoints.push_back(pt);
        }

        // ===== recompute objective =====
        double objective = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const double v_i = solution(VIndex(i));
            const double a_i = solution(AIndex(i));
            const double dv = v_i - config_.reference_speed;
            objective += wRef * dv * dv;
            objective += wAcc * a_i * a_i;
        }
        if (wJerk > 0.0)
        {
            const double jerkScale = wJerk / (dt * dt);
            for (int i = 0; i + 1 < n; ++i)
            {
                const double da = solution(AIndex(i + 1)) - solution(AIndex(i));
                objective += jerkScale * da * da;
            }
        }
        if (wProgress > 0.0)
        {
            const double ds_end = solution(SIndex(n - 1)) - drivable_area.upper_boundary.back().s - totalMargin;
            objective += wProgress * ds_end * ds_end;
        }
        output.objective = objective;
        output.Flag = QpSpeedOptimizerFallback::Success;
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
