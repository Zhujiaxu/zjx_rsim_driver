#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

namespace rsim_driver
{
    namespace path_matcher_detail
    {

        template <typename T, typename = void>
        struct HasHeading : std::false_type
        {
        };

        template <typename T>
        struct HasHeading<T, std::void_t<decltype(std::declval<T &>().hdg)>>
            : std::true_type
        {
        };

        inline std::size_t kMatchConfirmLookahead = 15;
        inline double DistanceSquared(double x0, double y0, double x1, double y1)
        {
            const double dx = x0 - x1;
            const double dy = y0 - y1;
            return dx * dx + dy * dy;
        }

        template <typename PointT>
        double HeadingAtIndex(const std::vector<PointT> &points, std::size_t index)
        {
            if (points.size() < 2)
            {
                if constexpr (HasHeading<PointT>::value)
                {
                    return points.empty() ? 0.0 : points[index].hdg;
                }
                else
                {
                    return 0.0;
                }
            }
            else
            {
                if constexpr (HasHeading<PointT>::value)
                {
                    return points[index].hdg;
                }
                else
                {
                    return 0.0;
                }
            }
        }

    } // namespace path_matcher_detail

    template <typename PointT>
    std::size_t FindMatchPointIndex(const std::vector<PointT> &points,
                                    const double &x,
                                    const double &y)
    {
        if (points.empty())
            return 0;

        std::size_t bestIndex = 0;
        std::size_t nonImprovingCount = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const double distance =
                path_matcher_detail::DistanceSquared(points[i].x, points[i].y, x, y);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = i;
                nonImprovingCount = 0;
                continue;
            }

            ++nonImprovingCount;
            if (nonImprovingCount >= path_matcher_detail::kMatchConfirmLookahead)
            {
                break;
            }
        }

        return bestIndex;
    }

    template <typename PointT>
    PointT FindProjectionPoint(const std::vector<PointT> &points,
                               double x,
                               double y)
    {
        if (points.empty())
            return PointT{};

        const std::size_t matchIndex = FindMatchPointIndex(points, x, y);
        PointT projection = points[matchIndex];

        const double heading = path_matcher_detail::HeadingAtIndex(points, matchIndex);
        const double tangentX = std::cos(heading);
        const double tangentY = std::sin(heading);
        const double dx = x - points[matchIndex].x;
        const double dy = y - points[matchIndex].y;
        const double scalar = dx * tangentX + dy * tangentY;

        projection.x = points[matchIndex].x + scalar * tangentX;
        projection.y = points[matchIndex].y + scalar * tangentY;
        if constexpr (path_matcher_detail::HasHeading<PointT>::value)
        {
            projection.hdg = heading;
        }

        return projection;
    }

} // namespace rsim_driver
