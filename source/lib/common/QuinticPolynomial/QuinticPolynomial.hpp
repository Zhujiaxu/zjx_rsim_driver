#pragma once

#include <Eigen/Dense>

#include <cmath>

namespace rsim_driver
{

struct QuinticBoundary
{
    double l = 0.0;
    double dl = 0.0;
    double ddl = 0.0;
};

class QuinticPolynomial1d
{
public:
    static bool Create(double deltaS,
                       const QuinticBoundary& start,
                       const QuinticBoundary& end,
                       QuinticPolynomial1d* polynomial)
    {
        if (polynomial == nullptr || !std::isfinite(deltaS) || deltaS <= 1e-9)
            return false;

        QuinticPolynomial1d result;
        result.a0_ = start.l;
        result.a1_ = start.dl;
        result.a2_ = 0.5 * start.ddl;

        const double t = deltaS;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;

        Eigen::Matrix3d matrix;
        matrix << t3, t4, t5,
                  3.0 * t2, 4.0 * t3, 5.0 * t4,
                  6.0 * t, 12.0 * t2, 20.0 * t3;

        Eigen::Vector3d rhs;
        rhs << end.l - (result.a0_ + result.a1_ * t + result.a2_ * t2),
               end.dl - (result.a1_ + 2.0 * result.a2_ * t),
               end.ddl - 2.0 * result.a2_;

        const Eigen::Vector3d coeff = matrix.householderQr().solve(rhs);
        if (!coeff.allFinite())
            return false;

        result.a3_ = coeff[0];
        result.a4_ = coeff[1];
        result.a5_ = coeff[2];
        *polynomial = result;
        return true;
    }

    double Evaluate(int order, double s) const
    {
        if (order <= 0)
        {
            return (((a5_ * s + a4_) * s + a3_) * s + a2_) * s * s +
                   a1_ * s + a0_;
        }
        if (order == 1)
        {
            return ((5.0 * a5_ * s + 4.0 * a4_) * s + 3.0 * a3_) * s * s +
                   2.0 * a2_ * s + a1_;
        }
        if (order == 2)
        {
            return ((20.0 * a5_ * s + 12.0 * a4_) * s + 6.0 * a3_) * s +
                   2.0 * a2_;
        }
        return 0.0;
    }

private:
    double a0_ = 0.0;
    double a1_ = 0.0;
    double a2_ = 0.0;
    double a3_ = 0.0;
    double a4_ = 0.0;
    double a5_ = 0.0;
};

}  // namespace rsim_driver
