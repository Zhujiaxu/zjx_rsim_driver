/*
 * PolynomialSolver — shared polynomial builders used by EMPlanner & SamplingPlanner.
 * All functions are free functions (no class dependency).
 */
#pragma once

#include <cmath>

namespace rsim_driver
{

// Solve 2x2 linear system via Cramer's rule.
inline bool Solve2x2(double a00, double a01, double b0,
                     double a10, double a11, double b1,
                     double& x0, double& x1)
{
    double det = a00 * a11 - a01 * a10;
    if (std::fabs(det) < 1e-9)
        return false;
    x0 = (b0 * a11 - a01 * b1) / det;
    x1 = (a00 * b1 - b0 * a10) / det;
    return true;
}

// Gauss-Jordan elimination with partial pivoting for a 3x4 augmented matrix.
inline bool Solve3x3(double a[3][4], double x[3])
{
    for (int i = 0; i < 3; ++i)
    {
        int pivot = i;
        for (int r = i + 1; r < 3; ++r)
        {
            if (std::fabs(a[r][i]) > std::fabs(a[pivot][i]))
                pivot = r;
        }
        if (std::fabs(a[pivot][i]) < 1e-9)
            return false;
        if (pivot != i)
        {
            for (int c = i; c < 4; ++c)
                std::swap(a[i][c], a[pivot][c]);
        }
        const double div = a[i][i];
        for (int c = i; c < 4; ++c)
            a[i][c] /= div;
        for (int r = 0; r < 3; ++r)
        {
            if (r == i)
                continue;
            const double factor = a[r][i];
            for (int c = i; c < 4; ++c)
                a[r][c] -= factor * a[i][c];
        }
    }
    x[0] = a[0][3];
    x[1] = a[1][3];
    x[2] = a[2][3];
    return true;
}

// Build quartic longitudinal polynomial: s(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4
// Initial: s0, v0, a0. Terminal: vT, aT.
inline bool BuildLongitudinalQuartic(double s0, double v0, double a0,
                                     double targetSpeed, double targetAccel,
                                     double T, double coeffs[5])
{
    if (T <= 1e-3)
        return false;
    coeffs[0] = s0;
    coeffs[1] = v0;
    coeffs[2] = 0.5 * a0;

    const double T2 = T * T;
    double a[2][3] = {
        {3.0 * T2, 4.0 * T2 * T, targetSpeed - coeffs[1] - 2.0 * coeffs[2] * T},
        {6.0 * T, 12.0 * T2, targetAccel - 2.0 * coeffs[2]},
    };
    const double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (std::fabs(det) < 1e-9)
        return false;
    coeffs[3] = (a[0][2] * a[1][1] - a[0][1] * a[1][2]) / det;
    coeffs[4] = (a[0][0] * a[1][2] - a[0][2] * a[1][0]) / det;
    return true;
}

// Build quintic lateral polynomial: d(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
// Initial: d0, v0, a0. Terminal: dT, vT, aT.
inline bool BuildLateralQuintic(double d0, double v0, double a0,
                                double dT, double vT, double aT,
                                double T, double coeffs[6])
{
    if (T <= 1e-3)
        return false;
    coeffs[0] = d0;
    coeffs[1] = v0;
    coeffs[2] = 0.5 * a0;

    double a[3][4] = {
        {T * T * T, T * T * T * T, T * T * T * T * T,
         dT - (coeffs[0] + coeffs[1] * T + coeffs[2] * T * T)},
        {3.0 * T * T, 4.0 * T * T * T, 5.0 * T * T * T * T,
         vT - (coeffs[1] + 2.0 * coeffs[2] * T)},
        {6.0 * T, 12.0 * T * T, 20.0 * T * T * T,
         aT - 2.0 * coeffs[2]},
    };
    double x[3] = {};
    if (!Solve3x3(a, x))
        return false;
    coeffs[3] = x[0];
    coeffs[4] = x[1];
    coeffs[5] = x[2];
    return true;
}

}  // namespace rsim_driver
