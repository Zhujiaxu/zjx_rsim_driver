/*
 * ReferenceLine — reference line data structures for planners.
 *
 * A ReferenceLine is a discrete sequence of ReferencePoints sampled along a
 * lane centreline, with cumulative chord-length s and pre-computed curvature.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

struct ReferencePoint
{
    double x   = 0.0;
    double y   = 0.0;
    double hdg = 0.0;  // heading (rad)
    double k   = 0.0;  // curvature (1/m)
    double dk  = 0.0;  // curvature derivative (1/m^2)
    double s   = 0.0;  // cumulative arc length along reference line
};

struct ReferenceLine
{
    std::vector<ReferencePoint> points;

    bool   empty() const { return points.empty(); }
    size_t size()  const { return points.size(); }

    // Frenet-to-world projection: given (s, d) where s is arc-length along
    // the reference line and d is lateral offset (positive = left), compute
    // the world (x, y, heading).  Uses linear interpolation between the two
    // nearest sample points.
    bool Eval(double s, double d, double* x, double* y, double* heading) const
    {
        if (points.empty())
            return false;

        if (s <= points.front().s)
        {
            *x = points.front().x + d * std::cos(points.front().hdg + M_PI_2);
            *y = points.front().y + d * std::sin(points.front().hdg + M_PI_2);
            *heading = points.front().hdg;
            return true;
        }
        if (s >= points.back().s)
        {
            *x = points.back().x + d * std::cos(points.back().hdg + M_PI_2);
            *y = points.back().y + d * std::sin(points.back().hdg + M_PI_2);
            *heading = points.back().hdg;
            return true;
        }

        // binary-search the segment containing s
        size_t lo = 0, hi = points.size() - 1;
        while (lo + 1 < hi)
        {
            size_t mid = (lo + hi) / 2;
            if (points[mid].s <= s)
                lo = mid;
            else
                hi = mid;
        }

        const ReferencePoint& p0 = points[lo];
        const ReferencePoint& p1 = points[hi];
        const double segLen = p1.s - p0.s;
        const double ratio  = (segLen > 1e-9) ? (s - p0.s) / segLen : 0.0;

        double interpX = p0.x + ratio * (p1.x - p0.x);
        double interpY = p0.y + ratio * (p1.y - p0.y);

        // heading interpolation with wrap-around handling
        double dh = p1.hdg - p0.hdg;
        while (dh > M_PI)  dh -= 2.0 * M_PI;
        while (dh < -M_PI) dh += 2.0 * M_PI;
        double interpHdg = p0.hdg + ratio * dh;

        *x = interpX + d * std::cos(interpHdg + M_PI_2);
        *y = interpY + d * std::sin(interpHdg + M_PI_2);
        *heading = interpHdg;
        return true;
    }

    // Curvature at arc-length s (linear interpolation between samples).
    double Curvature(double s) const
    {
        if (points.empty())
            return 0.0;
        if (s <= points.front().s)
            return points.front().k;
        if (s >= points.back().s)
            return points.back().k;

        size_t lo = 0, hi = points.size() - 1;
        while (lo + 1 < hi)
        {
            size_t mid = (lo + hi) / 2;
            if (points[mid].s <= s)
                lo = mid;
            else
                hi = mid;
        }

        const double segLen = points[hi].s - points[lo].s;
        const double ratio  = (segLen > 1e-9) ? (s - points[lo].s) / segLen : 0.0;
        return points[lo].k + ratio * (points[hi].k - points[lo].k);
    }
};

}  // namespace rsim_driver
