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


bool ValidPlanConfig(const DynamicPlanSpeedConfig &config)
{
    return IsFinite(config.time_step) &&
           config.time_step > kEpsilon &&
           config.time_step_count > 0;
}

std::vector<double> BuildTimeGrid(const DynamicPlanSpeedConfig &config)
{
    std::vector<double> grid;
    if (!ValidPlanConfig(config))
        return grid;
    grid.reserve(static_cast<std::size_t>(config.time_step_count) + 1);
    for (int i = 0; i <= config.time_step_count; ++i)
        grid.push_back(static_cast<double>(i) * config.time_step);
    return grid;
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
                                double *s_min_out, double *s_max_out)
{
    if (s_min_out == nullptr || s_max_out == nullptr)
        return false;

    const double dt = tout - tin;
    if (dt <= kEpsilon)
    {
        *s_min_out = sinmin;
        *s_max_out = sinmax;
        return true;
    }

    double alpha = (t - tin) / dt;
    alpha = std::max(0.0, std::min(1.0, alpha));

    *s_min_out = sinmin + alpha * (soutmin - sinmin);
    *s_max_out = sinmax + alpha * (soutmax - sinmax);
    return true;
}

}  // namespace

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
    const DynamicPlanSpeedPoint& start,
    const DynamicPlanSpeedConfig &plan_config,
    double reference_line_total_length,
    StDrivableArea *result) const
{
    if (result == nullptr)
        return false;

    StDrivableArea output;

    if (!ValidPlanConfig(plan_config) ||
        !IsFinite(reference_line_total_length) ||
        reference_line_total_length <= kEpsilon)
    {
        *result = output;
        return false;
    }

    const std::vector<double> time_grid = BuildTimeGrid(plan_config);
    if (time_grid.empty())
    {
        *result = output;
        return false;
    }

    const std::size_t n_steps = time_grid.size();

    output.lower_boundary.reserve(n_steps);
    output.upper_boundary.reserve(n_steps);
    for (double t : time_grid)
    {
        output.lower_boundary.push_back({0.0, t});
        output.upper_boundary.push_back({reference_line_total_length, t});
    }

    for (const CutInAndOutInfo &info : st_boundary_infos)
    {
        if (!IsValidStPolygon(info))
            continue;

        const double effective_tin = std::max(0.0, info.tin);

        if (effective_tin > info.tout + kEpsilon)
            continue;

        for (std::size_t i = 0; i < n_steps; ++i)
        {
            const double t = time_grid[i];

            if (t < effective_tin - kEpsilon || t > info.tout + kEpsilon)
                continue;

            double s_min_out = 0.0;
            double s_max_out = 0.0;
            if (!InterpolateObstacleSRange(t,
                                            info.tin, info.tout,
                                            info.sinmin, info.sinmax,
                                            info.soutmin, info.soutmax,
                                            &s_min_out, &s_max_out))
                continue;

            if (!IsFinite(s_min_out) || !IsFinite(s_max_out))
                continue;

            const double ego_approx_s = start.v * t;

            if (s_min_out + kEpsilon > ego_approx_s)
            {
                output.upper_boundary[i].s =
                    std::min(output.upper_boundary[i].s, s_min_out);
            }
            else
            {
                output.lower_boundary[i].s =
                    std::max(output.lower_boundary[i].s, s_max_out);
            }
        }
    }

    for (std::size_t i = 0; i < n_steps; ++i)
    {
        if (output.lower_boundary[i].s > output.upper_boundary[i].s)
        {
            *result = StDrivableArea{};
            return false;
        }
    }

    *result = std::move(output);
    return true;
}

}  // namespace rsim_driver
