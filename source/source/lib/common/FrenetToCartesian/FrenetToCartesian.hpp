#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

struct CartesianPathPoint
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double s = 0.0;
    double l = 0.0;
};

namespace frenet_to_cartesian_detail
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

inline double NormalizeAngle(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

template <typename RefPointT>
RefPointT InterpolateReferencePoint(const std::vector<RefPointT>& referencePoints,
                                    double s)
{
    if (referencePoints.empty())
        return RefPointT{};
    if (referencePoints.size() == 1 || s <= referencePoints.front().s)
        return referencePoints.front();
    if (s >= referencePoints.back().s)
        return referencePoints.back();

    for (std::size_t i = 1; i < referencePoints.size(); ++i)
    {
        const RefPointT& previous = referencePoints[i - 1];
        const RefPointT& next = referencePoints[i];
        if (s > next.s)
            continue;

        const double ds = next.s - previous.s;
        const double ratio = std::fabs(ds) > kEpsilon
                                 ? std::clamp((s - previous.s) / ds, 0.0, 1.0)
                                 : 0.0;

        RefPointT interpolated = previous;
        interpolated.x = previous.x + (next.x - previous.x) * ratio;
        interpolated.y = previous.y + (next.y - previous.y) * ratio;
        interpolated.hdg = NormalizeAngle(
            previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
        interpolated.s = s;
        return interpolated;
    }

    return referencePoints.back();
}

inline double SegmentHeading(const CartesianPathPoint& from,
                             const CartesianPathPoint& to,
                             double fallback)
{
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    if (dx * dx + dy * dy <= kEpsilon * kEpsilon)
        return fallback;
    return std::atan2(dy, dx);
}

inline void RecomputePathHeadings(std::vector<CartesianPathPoint>* path)
{
    if (path == nullptr || path->size() < 2)
        return;

    std::vector<CartesianPathPoint>& points = *path;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        if (i == 0)
        {
            points[i].heading = SegmentHeading(points[i], points[i + 1],
                                               points[i].heading);
        }
        else if (i + 1 == points.size())
        {
            points[i].heading = SegmentHeading(points[i - 1], points[i],
                                               points[i].heading);
        }
        else
        {
            points[i].heading = SegmentHeading(points[i - 1], points[i + 1],
                                               points[i].heading);
        }
    }
}

}  // namespace frenet_to_cartesian_detail

template <typename RefPointT>
bool FrenetPointToCartesian(const std::vector<RefPointT>& referencePoints,
                            double s,
                            double l,
                            CartesianPathPoint* cartesianPoint)
{
    if (cartesianPoint == nullptr || referencePoints.empty())
        return false;

    const RefPointT ref =
        frenet_to_cartesian_detail::InterpolateReferencePoint(referencePoints, s);
    const double normalX = -std::sin(ref.hdg);
    const double normalY = std::cos(ref.hdg);

    CartesianPathPoint point;
    point.x = ref.x + l * normalX;
    point.y = ref.y + l * normalY;
    point.heading = ref.hdg;
    point.s = s;
    point.l = l;

    *cartesianPoint = point;
    return true;
}

template <typename RefPointT, typename FrenetPointT>
bool FrenetPathToCartesian(const std::vector<RefPointT>& referencePoints,
                           const std::vector<FrenetPointT>& frenetPath,
                           std::vector<CartesianPathPoint>* cartesianPath)
{
    if (cartesianPath == nullptr)
        return false;

    cartesianPath->clear();
    if (referencePoints.empty() || frenetPath.empty())
        return false;

    cartesianPath->reserve(frenetPath.size());
    for (const FrenetPointT& frenetPoint : frenetPath)
    {
        CartesianPathPoint point;
        if (!FrenetPointToCartesian(referencePoints,
                                    frenetPoint.s,
                                    frenetPoint.l,
                                    &point))
        {
            cartesianPath->clear();
            return false;
        }
        cartesianPath->push_back(point);
    }

    frenet_to_cartesian_detail::RecomputePathHeadings(cartesianPath);
    return true;
}

}  // namespace rsim_driver
