#pragma once

#include "PathMatcher.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace rsim_driver
{

    struct StartPointFrenetState
    {
        double s = 0.0;
        double s_dot = 0.0;
        double l = 0.0;
        double l_dot = 0.0;
        double l_prime = 0.0;
        double l_double_prime = 0.0;
    };
    struct StaticAndVirtualObsFrenetState
    {
        int32_t id = 0;
        double s = 0.0;
        double l = 0.0;
        double length = 0.0;
        double width = 0.0;
    };
    struct DynamicObsFrenetState
    {
        int32_t id = 0;
        //double relangle = 0.0;
        double s = 0.0;
        double l = 0.0;
        double s_dot = 0.0;
        double l_dot = 0.0;
        double length = 0.0;
        double width = 0.0;
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

        inline bool IsFinite(const StartPointFrenetState &state)
        {
            return std::isfinite(state.s) &&
                   std::isfinite(state.s_dot) &&
                   // std::isfinite(state.s_ddot) &&
                   std::isfinite(state.l) &&
                   std::isfinite(state.l_dot) &&
                   std::isfinite(state.l_prime) &&
                   std::isfinite(state.l_double_prime);
        }
        inline bool IsFinite(const StaticAndVirtualObsFrenetState &state)
        {
            return std::isfinite(state.s) &&
                   std::isfinite(state.l);
        }

        inline bool IsFinite(const DynamicObsFrenetState &state)
        {
            return std::isfinite(state.s) &&
                   std::isfinite(state.l) &&
                   std::isfinite(state.s_dot) &&
                   std::isfinite(state.l_dot);
        }

        template <typename RefPointT, typename CartesianPointT>
        bool StaticObsFrenetTransformer(const std::vector<RefPointT> &referencePoints,
                                        const CartesianPointT &cartesianPoint,
                                        StaticAndVirtualObsFrenetState *frenetState)
        {
            if (frenetState == nullptr)
                return false;
            const std::size_t matchIndex =
                FindMatchPointIndex(referencePoints, cartesianPoint.x, cartesianPoint.y);
            const RefPointT &matchedPoint = referencePoints[matchIndex];
            const RefPointT projectionPoint =
                FindProjectionPoint(referencePoints, cartesianPoint.x, cartesianPoint.y);
            const double tangentX = std::cos(matchedPoint.hdg);
            const double tangentY = std::sin(matchedPoint.hdg);
            const double normalX = -std::sin(matchedPoint.hdg);
            const double normalY = std::cos(matchedPoint.hdg);
            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double lateralDx = cartesianPoint.x - projectionPoint.x;
            const double lateralDy = cartesianPoint.y - projectionPoint.y;

            StaticAndVirtualObsFrenetState obstacle;
            obstacle.id = cartesianPoint.id;
            obstacle.s = matchedPoint.s +
                         projectionDx * tangentX +
                         projectionDy * tangentY;
            obstacle.l = lateralDx * normalX + lateralDy * normalY;
            double relativeangle = NormalizeAngle(cartesianPoint.heading - matchedPoint.hdg);
            obstacle.length = cartesianPoint.length * std::fabs(std::cos(relativeangle));
            obstacle.width = cartesianPoint.width * std::fabs(std::sin(relativeangle));
            if (!IsFinite(obstacle))
            {
                std::cout << "【common】StaticObsFrenetTransformer: 感知静态障碍物结果参数无效" << std::endl;
                return false;
            }
            *frenetState = std::move(obstacle);
            return true;
        }
        template <typename RefPointT, typename CartesianPointT,typename VirtualSeed>
        bool VirtualObsFrenetTransformer(const std::vector<RefPointT> &referencePoints,
                                         const CartesianPointT &cartesianPoint,
                                         const VirtualSeed &seed,
                                         StaticAndVirtualObsFrenetState *frenetState)
        {
            seed.ttl--;
            if (frenetState == nullptr)
                return false;
            const std::size_t matchIndex =
                FindMatchPointIndex(referencePoints, cartesianPoint.x, cartesianPoint.y);
            const RefPointT &matchedPoint = referencePoints[matchIndex];
            const RefPointT projectionPoint =
                FindProjectionPoint(referencePoints, cartesianPoint.x, cartesianPoint.y);

            const double tangentX = std::cos(matchedPoint.hdg);
            const double tangentY = std::sin(matchedPoint.hdg);
            const double normalX = -std::sin(matchedPoint.hdg);
            const double normalY = std::cos(matchedPoint.hdg);
            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double lateralDx = cartesianPoint.x - projectionPoint.x;
            const double lateralDy = cartesianPoint.y - projectionPoint.y;

            StaticAndVirtualObsFrenetState obstacle;
            obstacle.id = cartesianPoint.id;
            obstacle.s = matchedPoint.s +
                         projectionDx * tangentX +
                         projectionDy * tangentY;
            obstacle.l = lateralDx * normalX + lateralDy * normalY;
            double relativeangle = NormalizeAngle(cartesianPoint.heading - matchedPoint.hdg);
            obstacle.length = cartesianPoint.length * std::fabs(std::cos(relativeangle)) + seed.longitudinal_buffer;
            obstacle.width = cartesianPoint.width * std::fabs(std::sin(relativeangle)) + seed.lateral_buffer;
            if (!IsFinite(obstacle))
            {
                std::cout << "【common】StaticObsFrenetTransformer: 感知虚拟障碍物结果参数无效" << std::endl;
                return false;
            }

            *frenetState = std::move(obstacle);
            return true;
        }

        template <typename RefPointT, typename CartesianPointT>
        bool DynamicObsFrenetTransformer(const std::vector<RefPointT> &referencePoints,
                                         const CartesianPointT &cartesianPoint,
                                         DynamicObsFrenetState *frenetState)
        {
            if (frenetState == nullptr || referencePoints.empty())
                return false;
            const std::size_t matchIndex =
                FindMatchPointIndex(referencePoints, cartesianPoint.x, cartesianPoint.y);
            const RefPointT &matchedPoint = referencePoints[matchIndex];
            const RefPointT projectionPoint =
                FindProjectionPoint(referencePoints, cartesianPoint.x, cartesianPoint.y);

            const double tangentX = std::cos(matchedPoint.hdg);
            const double tangentY = std::sin(matchedPoint.hdg);
            const double normalX = -std::sin(matchedPoint.hdg);
            const double normalY = std::cos(matchedPoint.hdg);
            const double projectionDx = projectionPoint.x - matchedPoint.x;
            const double projectionDy = projectionPoint.y - matchedPoint.y;
            const double lateralDx = cartesianPoint.x - projectionPoint.x;
            const double lateralDy = cartesianPoint.y - projectionPoint.y;

            DynamicObsFrenetState obstacle;
            obstacle.id = cartesianPoint.id;
            obstacle.s = matchedPoint.s +
                         projectionDx * tangentX +
                         projectionDy * tangentY;
            obstacle.l = lateralDx * normalX + lateralDy * normalY;
            double relativeangle = NormalizeAngle(cartesianPoint.heading - matchedPoint.hdg);
            //obstacle.relangle = relativeangle;
            if (1 - matchedPoint.k * obstacle.l <= kEpsilon)
            {
                std::cout << "【common】DynamicObsFrenetTransformer: 感知动态障碍物结果s_dot无穷" << std::endl;
                return false;
            }
            obstacle.s_dot = cartesianPoint.speed * std::cos(relativeangle) / (1 - matchedPoint.k * obstacle.l);
            obstacle.l_dot = cartesianPoint.speed * std::sin(relativeangle);
            obstacle.length = cartesianPoint.length * std::fabs(std::cos(relativeangle));
            obstacle.width = cartesianPoint.width * std::fabs(std::sin(relativeangle));
            if (!IsFinite(obstacle))
            {
                std::cout << "【common】DynamicObsFrenetTransformer: 感知动态障碍物结果参数无效" << std::endl;
                return false;
            }
            *frenetState = std::move(obstacle);
            return true;
        }

        template <typename RefPointT>
        bool InterpolateReferenceStateByS(const std::vector<RefPointT> &referencePoints,
                                          double s,
                                          const RefPointT *interpolated)
        {
            if (interpolated == nullptr)
                return false;
            if (referencePoints.size() == 1 || s < referencePoints.front().s)
            {
                std::cout << "【common】InterpolateReferenceStateByS: s is out of range" << std::endl;
                return false;
            }
            if (s > referencePoints.back().s)
            {
                std::cout << "【common】InterpolateReferenceStateByS: s is out of range" << std::endl;
                return false;
            }

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

                RefPointT point = previous;
                point.x = previous.x + (next.x - previous.x) * ratio;
                point.y = previous.y + (next.y - previous.y) * ratio;
                point.hdg = NormalizeAngle(
                    previous.hdg + NormalizeAngle(next.hdg - previous.hdg) * ratio);
                point.k = previous.k + (next.k - previous.k) * ratio;
                point.dk = previous.dk + (next.dk - previous.dk) * ratio;
                point.s = s;

                *interpolated = std::move(point);

                return true;
            }

            return false;
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
        bool StartPointFrenetStateTranformer(const std::vector<RefPointT> &referencePoints,
                                    const CartesianPointT &cartesianPoint,
                                    double curvature,
                                    StartPointFrenetState *frenetState)
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
            const RefPointT projectionReference;
            if (!InterpolateReferenceStateByS(referencePoints,
                                              refinedS,
                                              &projectionReference))
            {
                std::cout << "【common】CartesianPointToFrenet: 规划起始点转Frenet失败" << std::endl;
                return false;
            }

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
            // sDdot ;
            const double sDdot =
                (cartesianPoint.accel * cosDeltaTheta - cartesianPoint.speed * cartesianPoint.speed * curvature * sinDeltaTheta) / (oneMinusKappaRefL) +
                (referenceKappa * sDot * sDot * lPrime) / (oneMinusKappaRefL) +
                (referenceDkappa * sDot * l + referenceKappa * lDot) * sDot / oneMinusKappaRefL;

            // lDdot ;
            const double lDdot =
                cartesianPoint.accel * sinDeltaTheta +
                curvature * cosDeltaTheta * cartesianPoint.speed * cartesianPoint.speed -
                referenceKappa * sDot * cartesianPoint.speed * cosDeltaTheta;

            const double sDotSquared = sDot * sDot;
            const double lDoublePrime =
                sDotSquared <= kEpsilon
                    ? 0.0
                    : (lDdot - lPrime * sDdot) / sDotSquared;

            StartPointFrenetState state;
            state.s = refinedS;
            state.s_dot = sDot;
            // state.s_ddot = sDdot;
            state.l = l;
            state.l_dot = lDot;
            // state.lDdot = lDdot;
            state.l_prime = lPrime;
            state.l_double_prime = lDoublePrime;

            if (!IsFinite(state))
            {
                std::cout << "【common】CartesianPointToFrenet: 规划起始点转Frenet参数不合理" << std::endl;
                return false;
            }

            *frenetState = state;
            return true;
        }

    } // namespace cartesian_to_frenet_detail

    template <typename RefPointT, typename PlanningStartResultT>
    bool CartesianToFrenet(const std::vector<RefPointT> &referencePoints,
                           const PlanningStartResultT &planningStartResult,
                           StartPointFrenetState *frenetState)
    {
        return cartesian_to_frenet_detail::StartPointFrenetStateTranformer(
            referencePoints,
            planningStartResult.start_point,
            cartesian_to_frenet_detail::PlanningStartCurvature(planningStartResult),
            frenetState);
    }

} // namespace rsim_driver
