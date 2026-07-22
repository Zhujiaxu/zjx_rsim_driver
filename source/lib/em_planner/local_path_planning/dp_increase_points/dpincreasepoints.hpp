#include "DpPlanner.hpp"
#include "QuinticPolynomial.hpp"

namespace rsim_driver
{
    struct DPIncreasePointsConfig
    {
        double count = 2;
    };
    struct DpIncreasePointsResult
    {
        std::vector<DpPathPoint> path;
    };
    class DPIncreasePoints
    {
    public:
        explicit DPIncreasePoints(DPIncreasePointsConfig config = {});
        void SetConfig(const DPIncreasePointsConfig &config);
        const DPIncreasePointsConfig &config() const;
        bool increasepoints(const DpPlannerResult &result, DpIncreasePointsResult *newresult) const;

    private:
        DPIncreasePointsConfig config_;
    };
} // namespace rsim_driver