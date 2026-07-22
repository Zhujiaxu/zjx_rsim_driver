#include "StDrivableArea.hpp"

#include <algorithm>
#include <cmath>

namespace rsim_driver
{

    namespace
    {

        constexpr double kEpsilon = 1e-9;

        bool IsFinite(double value)
        {
            return std::isfinite(value);
        }

        bool ValidDrivableAreaConfig(const StDrivableAreaConfig &config)
        {
            return IsFinite(config.longitudinal_safety_buffer);
        }

        bool IsValidStPolygon(const CutInAndOutInfo &info)
        {
            if (!IsFinite(info.tin) && !IsFinite(info.tout) &&
                !IsFinite(info.sin) && !IsFinite(info.sout))
                return false;

            if (!IsFinite(info.tin) || !IsFinite(info.tout) ||
                !IsFinite(info.sinmin) || !IsFinite(info.sinmax) ||
                !IsFinite(info.soutmin) || !IsFinite(info.soutmax))
                return false;

            return true;
        }

        bool InterpolateObstacleSRange(double t,
                                       double tin, double tout,
                                       double sinmin, double sinmax,
                                       double soutmin, double soutmax,
                                       double *low_boundary, double *up_boundary)
        {
            if (low_boundary == nullptr || up_boundary == nullptr)
                return false;

            const double dt = tout - tin;
            if (dt <= kEpsilon)
            {
                *low_boundary = sinmin;
                *up_boundary = sinmax;
                return true;
            }

            double alpha = (t - tin) / dt;
            alpha = std::max(0.0, std::min(1.0, alpha));

            *low_boundary = sinmin + alpha * (soutmin - sinmin);
            *up_boundary = sinmax + alpha * (soutmax - sinmax);
            return true;
        }

    } // namespace

    StDrivableAreaBuilder::StDrivableAreaBuilder(const StDrivableAreaConfig &config)
        : config_(config)
    {
    }

    const StDrivableAreaConfig &StDrivableAreaBuilder::config() const
    {
        return config_;
    }

    void StDrivableAreaBuilder::SetConfig(const StDrivableAreaConfig &config)
    {
        config_ = config;
    }

    bool StDrivableAreaBuilder::Build(
        const std::vector<CutInAndOutInfo> &st_boundary_infos,
        const DynamicPlanSpeedResult *dpplan_result,
        StDrivableAreaResult *result) const
    {
        if (result == nullptr)
            return false;

        StDrivableAreaResult output;

        if (!ValidDrivableAreaConfig(config_) ||
            dpplan_result == nullptr)
        {
            output.Flag=StDrivableAreaFallback::Other;
            *result = std::move(output);
            return false;
        }

        const std::size_t n_steps = dpplan_result->stpoints.size();

        output.lower_boundary.reserve(n_steps);
        output.upper_boundary.reserve(n_steps);
        for (const DynamicPlanSpeedPoint &point : dpplan_result->stpoints)
        {
            output.lower_boundary.push_back({0.0, point.t});
            output.upper_boundary.push_back({dpplan_result->total_s, point.t});
        }

        for (const CutInAndOutInfo &info : st_boundary_infos)
        {
            if (!IsValidStPolygon(info))
                continue;

            for (std::size_t i = 0; i < n_steps; ++i)
            {
                const DynamicPlanSpeedPoint &point = dpplan_result->stpoints[i];
                const double t = point.t;

                if (t < info.tin - kEpsilon || t > info.tout + kEpsilon)
                    continue;

                double low_boundary = 0.0;
                double up_boundary = 0.0;
                if (!InterpolateObstacleSRange(t,
                                               info.tin, info.tout,
                                               info.sinmin, info.sinmax,
                                               info.soutmin, info.soutmax,
                                               &low_boundary, &up_boundary))
                    continue;

                if (!IsFinite(low_boundary) || !IsFinite(up_boundary))
                    continue;

                const double ego_approx_s = dpplan_result->stpoints[i].s;

                if (low_boundary + kEpsilon > ego_approx_s)
                {
                    output.upper_boundary[i].s =
                        std::min(output.upper_boundary[i].s, low_boundary);
                }
                else if (up_boundary - kEpsilon < ego_approx_s)
                {
                    output.lower_boundary[i].s =
                        std::max(output.lower_boundary[i].s, up_boundary);
                }
                else
                {
                    output.Flag = StDrivableAreaFallback::Other;
                    *result=std::move(output);
                    return false;
                }
            }
        }

        for (std::size_t i = 0; i < n_steps; ++i)
        {
            if (output.upper_boundary[i].s - output.lower_boundary[i].s < config_.longitudinal_safety_buffer)
            {
                output.Flag = StDrivableAreaFallback::Stop;
                *result=std::move(output);
                return false;
            }
        }
        output.Flag = StDrivableAreaFallback::Success;
        *result = std::move(output);
        return true;
    }

} // namespace rsim_driver
