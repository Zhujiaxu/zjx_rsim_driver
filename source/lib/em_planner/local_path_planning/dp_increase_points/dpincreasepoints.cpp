#include "dpincreasepoints.hpp"
namespace rsim_driver
{
    DPIncreasePoints::DPIncreasePoints(DPIncreasePointsConfig config) : config_(config)
    {
    }

    void DPIncreasePoints::SetConfig(const DPIncreasePointsConfig &config)
    {
        config_ = config;
    }

    const DPIncreasePointsConfig &DPIncreasePoints::config() const
    {
        return config_;
    }

    bool rsim_driver::DPIncreasePoints::increasepoints(const DpPlannerResult &result, DpIncreasePointsResult *newresult) const
    {
        // --- input guards ---
        if (newresult == nullptr)
            return false;

        newresult->path.clear();

        if (config_.count < 1)
            return false;

        if (result.path.size() < 2)
            return false;

        // Validate all points: finite and s strictly increasing
        for (size_t i = 0; i < result.path.size(); ++i)
        {
            const auto &p = result.path[i];
            if (!std::isfinite(p.s) || !std::isfinite(p.l) ||
                !std::isfinite(p.l_prime) || !std::isfinite(p.l_double_prime))
            {
                return false;
            }
            if (i > 0 && p.s <= result.path[i - 1].s)
            {
                return false;
            }
        }

        newresult->path.clear();



        for (size_t i = 0; i < result.path.size() - 1; ++i)
        {
            const auto &startpoint = result.path[i];
            const auto &endpoint = result.path[i + 1];
            double delta_s = endpoint.s - startpoint.s;
            if (delta_s <= 1e-9)
            {
                newresult->path.clear();
                return false;
            }
            double s_step = delta_s / config_.count;

            QuinticBoundary start{startpoint.l, startpoint.l_prime, startpoint.l_double_prime};
            QuinticBoundary end{endpoint.l, endpoint.l_prime, endpoint.l_double_prime};

            QuinticPolynomial1d polynomial;
            if (!QuinticPolynomial1d::Create(delta_s, start, end, &polynomial))
            {
                newresult->path.clear();
                return false;
            }

            for (int j = 0; j <config_.count; ++j)
            {
                double s = j * s_step;
                DpPathPoint new_point;
                new_point.s = startpoint.s + s;
                new_point.l = polynomial.Evaluate(0, s);
                new_point.l_prime = polynomial.Evaluate(1, s);
                new_point.l_double_prime = polynomial.Evaluate(2, s);
                newresult->path.push_back(new_point);
            }
        }
        newresult->path.push_back(result.path.back()); // Add the last point from the original path

        return true;
    }

} // namespace rsim_driver
