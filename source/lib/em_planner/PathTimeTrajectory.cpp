/*
 * PathTimeTrajectory implementation.
 */
#include "PathTimeTrajectory.hpp"
#include "PolynomialSolver.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

namespace
{

// Return the QpPathPoint nearest to a given s (simple linear scan, fine for small arrays).
const QpPathPoint* PathAt(const std::vector<QpPathPoint>& path, double s)
{
    if (path.empty())
        return nullptr;
    if (s <= path.front().s)
        return &path.front();
    if (s >= path.back().s)
        return &path.back();

    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        if (s >= path[i].s && s <= path[i + 1].s)
        {
            double segLen = path[i + 1].s - path[i].s;
            double ratio  = (segLen > 1e-9) ? (s - path[i].s) / segLen : 0.0;
            // return the latter point for simplicity (or we could interpolate)
            return (ratio < 0.5) ? &path[i] : &path[i + 1];
        }
    }
    return &path.back();
}

}  // namespace

bool PathTimeTrajectory::Combine(
    const std::vector<QpPathPoint>& qpPath,
    const std::vector<QpSpeedPoint>& qpSpeed,
    double simTime,
    FrenetTrajectory* frenet,
    PlannedTrajectory* trajectory)
{
    if (qpPath.size() < 2 || qpSpeed.size() < 2)
        return false;
    if (!frenet || !trajectory)
        return false;

    const double T = qpSpeed.back().t;
    if (T <= 1e-3)
        return false;

    // --- Build FrenetTrajectory from boundary conditions ---
    const QpSpeedPoint& sp0 = qpSpeed.front();
    const QpSpeedPoint& spN = qpSpeed.back();

    // s(t) quartic: initial s0, v0, a0; terminal vT, aT=0
    if (!BuildLongitudinalQuartic(sp0.s, std::max(0.0, sp0.v), sp0.a,
                                  std::max(0.0, spN.v), 0.0,
                                  T, frenet->sCoeffs))
    {
        // fallback: use a simpler third-order fit
        frenet->sCoeffs[0] = sp0.s;
        frenet->sCoeffs[1] = sp0.v;
        frenet->sCoeffs[2] = 0.5 * sp0.a;
        frenet->sCoeffs[3] = 0.0;
        frenet->sCoeffs[4] = 0.0;
    }

    // d(t) quintic: use path's lateral info
    const QpPathPoint& pp0 = qpPath.front();
    const QpPathPoint& ppN = qpPath.back();

    // lateral speed at start: dl * s_d
    double d0_dot  = pp0.dl * std::max(0.1, sp0.v);
    double d0_ddot = pp0.ddl * sp0.v * sp0.v + pp0.dl * sp0.a;
    double dT_dot  = ppN.dl * std::max(0.1, spN.v);
    double dT_ddot = ppN.ddl * spN.v * spN.v + ppN.dl * spN.a;

    if (!BuildLateralQuintic(pp0.l, d0_dot, d0_ddot,
                             ppN.l, dT_dot, dT_ddot,
                             T, frenet->dCoeffs))
    {
        frenet->dCoeffs[0] = pp0.l;
        frenet->dCoeffs[1] = d0_dot;
        frenet->dCoeffs[2] = 0.5 * d0_ddot;
        frenet->dCoeffs[3] = 0.0;
        frenet->dCoeffs[4] = 0.0;
        frenet->dCoeffs[5] = 0.0;
    }

    frenet->T     = T;
    frenet->valid = true;
    frenet->cost  = 0.0;

    // --- Build PlannedTrajectory by sampling ---
    *trajectory = PlannedTrajectory{};
    const double sampleDt = std::max(0.1, T / static_cast<double>(PlannedTrajectory::MAX_POINTS - 1));
    int count = 0;
    for (double t = 0.0; t <= T + 1e-6 && count < PlannedTrajectory::MAX_POINTS; t += sampleDt)
    {
        TrajectoryPoint& p = trajectory->points[count];
        p.t   = t;
        p.s   = frenet->EvalS(t);
        p.d   = frenet->EvalD(t);

        double sDot  = frenet->EvalSdot(t);
        double sDDot = frenet->EvalSddot(t);
        double dDot  = frenet->EvalDdot(t);
        double dDDot = frenet->EvalDddot(t);

        p.speed   = std::max(0.0, sDot);
        p.heading = std::atan2(dDot, std::max(0.1, sDot));

        double denom = std::pow(std::max(0.1, sDot * sDot + dDot * dDot), 1.5);
        p.curvature = (denom > 1e-6) ? std::fabs(sDot * dDDot - dDot * sDDot) / denom : 0.0;

        ++count;
    }

    trajectory->numPoints = count;
    trajectory->startTime = simTime;
    return count >= 2;
}

}  // namespace rsim_driver
