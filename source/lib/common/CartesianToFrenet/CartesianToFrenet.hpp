#pragma once

#include "../PathMatcher/PathMatcher.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

    struct CartesianFrenetState
    {
        double s = 0.0;
        double s_dot = 0.0;
        double s_ddot = 0.0;
        double l = 0.0;
        double l_prime = 0.0;
        double l_double_prime = 0.0;
    };

    namespace cartesian_to_frenet_detail
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

        inline bool IsFinite(const CartesianFrenetState &state)
        {
            return std::isfinite(state.s) &&
                   std::isfinite(state.s_dot) &&
                   std::isfinite(state.s_ddot) &&
                   std::isfinite(state.l) &&
                   std::isfinite(state.l_prime) &&
                   std::isfinite(state.l_double_prime);
        }

    } // namespace cartesian_to_frenet_detail

    template <typename RefPointT, typename CartesianPointT>
    bool CartesianToFrenet(const std::vector<RefPointT> &referencePoints,
                           const CartesianPointT &cartesianPoint,
                           CartesianFrenetState *frenetState)
    {
        if (frenetState == nullptr || referencePoints.empty())
            return false;

        const std::size_t matchIndex =
            FindMatchPointIndex(referencePoints, cartesianPoint.x, cartesianPoint.y);
        const RefPointT &matchedPoint = referencePoints[matchIndex];
        const RefPointT projectionPoint =
            FindProjectionPoint(referencePoints, cartesianPoint.x, cartesianPoint.y);

        const double refHeading = matchedPoint.hdg;
        const double refTangentX = std::cos(refHeading);
        const double refTangentY = std::sin(refHeading);
        const double refNormalX = -std::sin(refHeading);
        const double refNormalY = std::cos(refHeading);

        const double projectionDx = projectionPoint.x - matchedPoint.x;
        const double projectionDy = projectionPoint.y - matchedPoint.y;
        const double refinedS = matchedPoint.s +
                                projectionDx * refTangentX +
                                projectionDy * refTangentY;
        const double dx = cartesianPoint.x - matchedPoint.x;
        const double dy = cartesianPoint.y - matchedPoint.y;
        const double l = dx * refNormalX + dy * refNormalY;
        if (dx * refTangentX + dy * refTangentY > 0)
        {
            const double projectionHeading = matchIndex + 1 == referencePoints.size()
                                                 ? referencePoints[matchIndex].hdg
                                                 : refHeading + (refHeading + referencePoints[matchIndex + 1].hdg) * 0.5;
        }
        const double projectionHeading = projectionPoint.hdg;

        const double deltaTheta = cartesian_to_frenet_detail::NormalizeAngle(
            cartesianPoint.heading - projectionHeading);
        const double cosDeltaTheta = std::cos(deltaTheta);
        if (std::fabs(cosDeltaTheta) <= cartesian_to_frenet_detail::kEpsilon)
            return false;

        const double tanDeltaTheta = std::tan(deltaTheta);
        const double oneMinusKappaRefL = 1.0 - matchedPoint.k * l;
        if (std::fabs(oneMinusKappaRefL) <= cartesian_to_frenet_detail::kEpsilon)
            return false;

        const double lPrime = oneMinusKappaRefL * tanDeltaTheta;
        const double kappaRefLPrime = matchedPoint.dk * l + matchedPoint.k * lPrime;
        const double deltaThetaPrime =
            oneMinusKappaRefL * cartesianPoint.curvature / cosDeltaTheta - matchedPoint.k;
        const double lDoublePrime =
            -kappaRefLPrime * tanDeltaTheta +
            oneMinusKappaRefL / (cosDeltaTheta * cosDeltaTheta) *
                (cartesianPoint.curvature * oneMinusKappaRefL / cosDeltaTheta -
                 matchedPoint.k);

        CartesianFrenetState state;
        state.s = refinedS;
        state.s_dot = cartesianPoint.speed * cosDeltaTheta / oneMinusKappaRefL;
        state.s_ddot =
            (cartesianPoint.accel * cosDeltaTheta -
             state.s_dot * state.s_dot *
                 (lPrime * deltaThetaPrime - kappaRefLPrime)) /
            oneMinusKappaRefL;
        state.l = l;
        state.l_prime = lPrime;
        state.l_double_prime = lDoublePrime;

        if (!cartesian_to_frenet_detail::IsFinite(state))
            return false;

        *frenetState = state;
        return true;
    }

} // namespace rsim_driver
