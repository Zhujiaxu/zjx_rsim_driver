#pragma once

#include <algorithm>
#include <array>
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

        std::array<std::array<double, 4>, 3> matrix = {{
            {{t3,  t4,   t5,   end.l - (result.a0_ + result.a1_ * t + result.a2_ * t2)}},
            {{3.0 * t2, 4.0 * t3, 5.0 * t4, end.dl - (result.a1_ + 2.0 * result.a2_ * t)}},
            {{6.0 * t,  12.0 * t2, 20.0 * t3, end.ddl - 2.0 * result.a2_}},
        }};

        if (!Solve3x3(&matrix))
            return false;

        result.a3_ = matrix[0][3];
        result.a4_ = matrix[1][3];
        result.a5_ = matrix[2][3];
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
    static bool Solve3x3(std::array<std::array<double, 4>, 3>* matrix)
    {
        if (matrix == nullptr)
            return false;

        auto& m = *matrix;
        for (int col = 0; col < 3; ++col)
        {
            int pivot = col;
            for (int row = col + 1; row < 3; ++row)
            {
                if (std::fabs(m[row][col]) > std::fabs(m[pivot][col]))
                    pivot = row;
            }
            if (std::fabs(m[pivot][col]) <= 1e-12)
                return false;
            if (pivot != col)
                std::swap(m[pivot], m[col]);

            const double divisor = m[col][col];
            for (int j = col; j < 4; ++j)
                m[col][j] /= divisor;

            for (int row = 0; row < 3; ++row)
            {
                if (row == col)
                    continue;
                const double factor = m[row][col];
                for (int j = col; j < 4; ++j)
                    m[row][j] -= factor * m[col][j];
            }
        }
        return true;
    }

    double a0_ = 0.0;
    double a1_ = 0.0;
    double a2_ = 0.0;
    double a3_ = 0.0;
    double a4_ = 0.0;
    double a5_ = 0.0;
};

}  // namespace rsim_driver
