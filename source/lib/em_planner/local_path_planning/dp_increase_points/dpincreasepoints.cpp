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

    bool rsim_driver::DPIncreasePoints::increasepoints(DpPlannerResult *result, DpPlannerResult *newresult) const
    {
        for (int i = 0; i < result->path.size() - 1; ++i)
        {
            const auto &startpoint = result->path[i];
            const auto &endpoint = result->path[i + 1];
            double delta_s = endpoint.s - startpoint.s;
            double s_step = delta_s / config_.count; // Calculate the step size based on the number of points to insert
            QuinticBoundary start{startpoint.l, startpoint.l_prime, startpoint.l_double_prime};
            QuinticBoundary end{endpoint.l, endpoint.l_prime, endpoint.l_double_prime};

            QuinticPolynomial1d polynomial;
            if (!QuinticPolynomial1d::Create(delta_s, start, end, &polynomial))
            {
                return false;
            }

            for (int i = 0; i < config_.count; i++)
            {
                double s = i * s_step;
                DpPathPoint new_point;
                new_point.s = startpoint.s + s;
                new_point.l = polynomial.Evaluate(0, s);
                new_point.l_prime = polynomial.Evaluate(1, s);
                new_point.l_double_prime = polynomial.Evaluate(2, s);
                newresult->path.push_back(new_point);
            }
        }
        newresult->path.push_back(result->path.back()); // Add the last point from the original path
        newresult->dpsuccess = result->dpsuccess;
        newresult->total_cost = result->total_cost;
        return true;
    }

} // namespace rsim_driver
