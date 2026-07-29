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
        double kappa = 0.0;
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
        RefPointT InterpolateReferencePoint(const std::vector<RefPointT> &referencePoints,
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
                const RefPointT &previous = referencePoints[i - 1];
                const RefPointT &next = referencePoints[i];
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
                interpolated.k = previous.k + (next.k - previous.k) * ratio;
                interpolated.dk= previous.dk + (next.dk - previous.dk) * ratio;
                interpolated.s = s;
                return interpolated;
            }

            return referencePoints.back();
        }

    } // namespace frenet_to_cartesian_detail

    template <typename RefPointT, typename FrenetPointT>
    bool FrenetPointToCartesian(const std::vector<RefPointT> &referencePoints,
                                FrenetPointT frenetPoint,
                                CartesianPathPoint *cartesianPoint)
    {
        if (cartesianPoint == nullptr || referencePoints.empty())
            return false;

        const RefPointT ref =
            frenet_to_cartesian_detail::InterpolateReferencePoint(referencePoints, frenetPoint.s);
        if (!std::isfinite(ref.x) || !std::isfinite(ref.y) ||
            !std::isfinite(ref.hdg) || !std::isfinite(ref.k) ||
            !std::isfinite(ref.dk) || !std::isfinite(frenetPoint.s) ||
            !std::isfinite(frenetPoint.l) ||
            !std::isfinite(frenetPoint.l_prime) ||
            !std::isfinite(frenetPoint.l_double_prime))
        {
            return false;
        }
        const double normalX = -std::sin(ref.hdg);
        const double normalY = std::cos(ref.hdg);
        const double oneMinusKappaRefL = 1.0 - ref.k * frenetPoint.l;
        if (std::fabs(oneMinusKappaRefL) <=
            frenet_to_cartesian_detail::kEpsilon)
        {
            return false;
        }
        const double deltaTheta =
            std::atan2(frenetPoint.l_prime, oneMinusKappaRefL);
        const double cosDeltaTheta = std::cos(deltaTheta);
        const double tanDeltaTheta = std::tan(deltaTheta);

        CartesianPathPoint point;
        point.x = ref.x + frenetPoint.l * normalX;
        point.y = ref.y + frenetPoint.l * normalY;
        point.heading = frenet_to_cartesian_detail::NormalizeAngle(
            ref.hdg + deltaTheta);
        point.kappa =
            ((frenetPoint.l_double_prime +
              (ref.dk * frenetPoint.l + ref.k * frenetPoint.l_prime) *
                  tanDeltaTheta) *
                 cosDeltaTheta * cosDeltaTheta / oneMinusKappaRefL +
             ref.k) *
            cosDeltaTheta / oneMinusKappaRefL;
        /*point.v = frenetPoint.s_prime * std::sqrt(std::pow(1 - ref.k * frenetPoint.l, 2) + std::pow(frenetPoint.l_prime, 2));
        point.a = frenetPoint.s_double_prime * std::sqrt(std::pow(1 - ref.k * frenetPoint.l, 2) + std::pow(frenetPoint.l_prime, 2)) +
                  frenetPoint.s_prime * ((1 - ref.k * frenetPoint.l) * (-ref.k * frenetPoint.l_prime) + frenetPoint.l_double_prime) /
                      std::sqrt(std::pow(1 - ref.k * frenetPoint.l, 2) + std::pow(frenetPoint.l_prime, 2));
        */
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.heading) || !std::isfinite(point.kappa))
        {
            return false;
        }
        *cartesianPoint = point;
        return true;
    }

    template <typename RefPointT, typename FrenetPointT>
    bool FrenetPathToCartesian(const std::vector<RefPointT> &referencePoints,
                               const std::vector<FrenetPointT> &frenetPath,
                               std::vector<CartesianPathPoint> *cartesianPath)
    {
        if (cartesianPath == nullptr)
            return false;

        cartesianPath->clear();
        if (referencePoints.empty() || frenetPath.empty())
            return false;

        cartesianPath->reserve(frenetPath.size());
        for (const FrenetPointT &frenetPoint : frenetPath)
        {

            CartesianPathPoint point;
            if (!FrenetPointToCartesian(referencePoints,
                                        frenetPoint,
                                        &point))
            {
                cartesianPath->clear();
                return false;
            }
            cartesianPath->push_back(point);
        }

        return true;
    }

} // namespace rsim_driver
