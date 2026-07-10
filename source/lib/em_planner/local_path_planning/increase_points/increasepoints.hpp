#include "DpPlanner.hpp"
#include "QuinticPolynomial.hpp"

namespace rsim_driver
{
    struct IncreasePointsConfig
    {
        double count = 2;
    };
    class IncreasePoints
    {
    public:
        explicit IncreasePoints(IncreasePointsConfig config = {});
        void SetConfig(const IncreasePointsConfig &config);
        const IncreasePointsConfig &config() const;
        bool increasepoints(DpPlannerResult *result, DpPlannerResult *newresult) const;

    private:
        IncreasePointsConfig config_;
    };
} // namespace rsim_driver