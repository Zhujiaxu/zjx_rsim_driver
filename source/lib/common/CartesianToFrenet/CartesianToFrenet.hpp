#pragma once

#include "PathMatcher.hpp"

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
        double ldot= 0.0;   
        double l_prime = 0.0;
        double l_double_prime = 0.0;
        double curvature = 0.0;
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
                   std::isfinite(state.l_double_prime) &&
                   std::isfinite(state.curvature);
        }

        template <typename RefPointT>
        RefPointT InterpolateReferenceStateByS(const std::vector<RefPointT> &referencePoints,
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
                                         ? (s - previous.s) / ds
                                         : 0.0;

                RefPointT interpolated = previous;
                interpolated.x = previous.x + (next.x - previous.x) * ratio;
                interpolated.y = previous.y + (next.y - previous.y) * ratio;
                interpolated.hdg = NormalizeAngle(
                    previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
                interpolated.k = previous.k + (next.k - previous.k) * ratio;
                interpolated.dk = previous.dk + (next.dk - previous.dk) * ratio;
                interpolated.s = s;
                return interpolated;
            }

            return referencePoints.back();
        }

        template <typename PlanningStartResultT>
        double PlanningStartCurvature(const PlanningStartResultT &planningStartResult)
        {
            const auto &cartesianPoint = planningStartResult.start_point;
            using SourceT = decltype(cartesianPoint.source);
            if (cartesianPoint.source == SourceT::KinematicExtrapolation)
                return 0.0;

            return planningStartResult.start_curvature;
        }

        template <typename RefPointT, typename CartesianPointT>
        bool CartesianPointToFrenet(const std::vector<RefPointT> &referencePoints,
                                    const CartesianPointT &cartesianPoint,
                                    double curvature,
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

            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double refinedS = matchedPoint.s +
                                    projectionDx * refTangentX +
                                    projectionDy * refTangentY;
            const RefPointT projectionReference =
                InterpolateReferenceStateByS(referencePoints,
                                             refinedS);

            const double projectionNormalX = -std::sin(projectionReference.hdg);
            const double projectionNormalY = std::cos(projectionReference.hdg);
            const double lateralDx = cartesianPoint.x - projectionPoint.x;
            const double lateralDy = cartesianPoint.y - projectionPoint.y;
            const double l = lateralDx * projectionNormalX + lateralDy * projectionNormalY;
            const double deltaTheta = NormalizeAngle(
                cartesianPoint.heading - projectionReference.hdg);
            const double cosDeltaTheta = std::cos(deltaTheta);
            if (std::fabs(cosDeltaTheta) <= kEpsilon)
                return false;

            const double tanDeltaTheta = std::tan(deltaTheta);
            const double referenceKappa = projectionReference.k;
            const double referenceDkappa = projectionReference.dk;
            const double oneMinusKappaRefL = 1.0 - referenceKappa * l;
            if (std::fabs(oneMinusKappaRefL) <= kEpsilon)
                return false;

            const double sinDeltaTheta = std::sin(deltaTheta);
            const double lPrime = oneMinusKappaRefL * tanDeltaTheta;
            const double sDot =
                cartesianPoint.speed * cosDeltaTheta / oneMinusKappaRefL;
            const double lDot = cartesianPoint.speed * sinDeltaTheta;
            const double sDdot =
                ((cartesianPoint.accel * cosDeltaTheta - cartesianPoint.speed * cartesianPoint.speed * referenceKappa * cosDeltaTheta +
                 referenceKappa * cartesianPoint.speed * sDot * sinDeltaTheta) *
                    (1 - referenceKappa * l) +
                cartesianPoint.speed * cosDeltaTheta *
                    (referenceKappa * lDot + referenceDkappa * l)) /
                (oneMinusKappaRefL * oneMinusKappaRefL);
            // lDdot ;
            const double lDdot =
                cartesianPoint.accel * sinDeltaTheta +
                referenceKappa * cosDeltaTheta * cartesianPoint.speed * cartesianPoint.speed -
                referenceKappa * sDot * cartesianPoint.speed;

            const double sDotSquared = sDot * sDot;
            const double lDoublePrime =
                sDotSquared <= kEpsilon
                    ? 0.0
                    : (lDdot - lPrime * sDdot) / sDotSquared;

            CartesianFrenetState state;
            state.s = refinedS;
            state.s_dot = sDot;
            state.s_ddot = sDdot;
            state.l = l;
            state.ldot = lDot;
            state.l_prime = lPrime;
            state.l_double_prime = lDoublePrime;
            state.curvature = curvature;

            if (!IsFinite(state))
                return false;

            *frenetState = state;
            return true;
        }

    } // namespace cartesian_to_frenet_detail

    template <typename RefPointT, typename PlanningStartResultT>
    bool CartesianToFrenet(const std::vector<RefPointT> &referencePoints,
                           const PlanningStartResultT &planningStartResult,
                           CartesianFrenetState *frenetState)
    {
        return cartesian_to_frenet_detail::CartesianPointToFrenet(
            referencePoints,
            planningStartResult.start_point,
            cartesian_to_frenet_detail::PlanningStartCurvature(planningStartResult),
            frenetState);
    }

} // namespace rsim_driver
