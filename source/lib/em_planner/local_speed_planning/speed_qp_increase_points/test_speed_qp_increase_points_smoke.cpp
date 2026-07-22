#include "SpeedQpIncreasePoints.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

namespace
{

bool Require(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DynamicPlanSpeedPoint EvaluateConstantJerk(
    const rsim_driver::DynamicPlanSpeedPoint& start,
    double time,
    double jerk)
{
    const double offset = time - start.t;
    const double offsetSquared = offset * offset;
    return {
        time,
        start.s + start.v * offset + 0.5 * start.a * offsetSquared +
            jerk * offsetSquared * offset / 6.0,
        start.v + start.a * offset + 0.5 * jerk * offsetSquared,
        start.a + jerk * offset,
    };
}

bool SamePoint(const rsim_driver::DynamicPlanSpeedPoint& actual,
               const rsim_driver::DynamicPlanSpeedPoint& expected)
{
    return actual.t == expected.t && actual.s == expected.s &&
           actual.v == expected.v && actual.a == expected.a;
}

rsim_driver::QpSpeedOptimizerResult MakeQpResult(
    std::vector<rsim_driver::DynamicPlanSpeedPoint> points)
{
    rsim_driver::QpSpeedOptimizerResult result;
    result.Flag = rsim_driver::QpSpeedOptimizerFallback::Success;
    result.stpoints = std::move(points);
    return result;
}

}  // namespace

int main()
{
    bool ok = true;

    const std::vector<rsim_driver::DynamicPlanSpeedPoint> constantSpeed = {
        {0.0, 0.0, 1.0, 0.0},
        {0.8, 0.8, 1.0, 0.0},
        {1.0, 1.0, 1.0, 0.0},
    };

    {
        rsim_driver::QpSpeedIncreasePoints densifier;
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;
        ok &= Require(densifier.config().count == 4,
                      "default subdivision count should be four");
        ok &= Require(densifier.increasepoints(MakeQpResult(constantSpeed),
                                               &output),
                      "constant-speed trajectory should densify");
        ok &= Require(output.size() == 9U,
                      "two intervals should produce nine points");
        if (output.size() == 9U)
        {
            const double expectedTimes[] = {
                0.0, 0.2, 0.4, 0.6, 0.8, 0.85, 0.9, 0.95, 1.0,
            };
            for (std::size_t i = 0; i < output.size(); ++i)
            {
                ok &= Require(Near(output[i].t, expectedTimes[i]),
                              "each coarse interval should be divided equally");
                ok &= Require(Near(output[i].s, output[i].t) &&
                                  Near(output[i].v, 1.0) &&
                                  Near(output[i].a, 0.0),
                              "constant-speed interpolation should remain exact");
            }
            ok &= Require(SamePoint(output[0], constantSpeed[0]) &&
                              SamePoint(output[4], constantSpeed[1]) &&
                              SamePoint(output[8], constantSpeed[2]),
                          "coarse endpoints should be preserved exactly");
        }
    }

    {
        rsim_driver::QpSpeedIncreasePoints densifier({2});
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;
        ok &= Require(densifier.config().count == 2,
                      "constructor config should be observable");
        densifier.SetConfig({1});
        ok &= Require(densifier.config().count == 1 &&
                          densifier.increasepoints(MakeQpResult(constantSpeed),
                                                   &output) &&
                          output.size() == constantSpeed.size(),
                      "count one should preserve the coarse point sequence");
        if (output.size() == constantSpeed.size())
        {
            for (std::size_t i = 0; i < output.size(); ++i)
            {
                ok &= Require(SamePoint(output[i], constantSpeed[i]),
                              "count one should preserve every coarse point");
            }
        }
    }

    {
        const double jerk = 2.0;
        const rsim_driver::DynamicPlanSpeedPoint start{0.0, 0.0, 0.0, 0.0};
        const rsim_driver::DynamicPlanSpeedPoint end =
            EvaluateConstantJerk(start, 1.0, jerk);
        rsim_driver::QpSpeedIncreasePoints densifier({2});
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;
        ok &= Require(densifier.increasepoints(MakeQpResult({start, end}),
                                               &output) &&
                          output.size() == 3U,
                      "constant-jerk trajectory should densify");
        if (output.size() == 3U)
        {
            const auto expected = EvaluateConstantJerk(start, 0.5, jerk);
            ok &= Require(Near(output[1].s, expected.s) &&
                              Near(output[1].v, expected.v) &&
                              Near(output[1].a, expected.a),
                          "inserted point should follow constant-jerk kinematics");
            ok &= Require(SamePoint(output.back(), end),
                          "constant-jerk endpoint should be preserved exactly");
        }
    }

    {
        rsim_driver::QpSpeedIncreasePoints densifier;
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output = {
            {99.0, 99.0, 99.0, 99.0},
        };
        ok &= Require(!densifier.increasepoints(MakeQpResult({}), &output) &&
                          output.empty(),
                      "empty input should fail and clear stale output");
        ok &= Require(!densifier.increasepoints(
                          MakeQpResult({constantSpeed.front()}), &output) &&
                          output.empty(),
                      "single-point input should fail and clear stale output");
        ok &= Require(!densifier.increasepoints(MakeQpResult(constantSpeed),
                                                nullptr),
                      "null output should fail");
    }

    {
        rsim_driver::QpSpeedIncreasePoints densifier({0});
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;
        ok &= Require(!densifier.increasepoints(MakeQpResult(constantSpeed),
                                                &output) &&
                          output.empty(),
                      "non-positive subdivision count should fail");
    }

    {
        rsim_driver::QpSpeedIncreasePoints densifier;
        std::vector<rsim_driver::DynamicPlanSpeedPoint> invalid = constantSpeed;
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;

        invalid[1].t = invalid[0].t;
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output),
                      "duplicate time should fail");

        invalid = constantSpeed;
        invalid[1].t = invalid[0].t - 0.1;
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output),
                      "backward time should fail");

        invalid = constantSpeed;
        invalid[1].s += 0.1;
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output),
                      "kinematically inconsistent endpoint should fail");

        invalid = constantSpeed;
        invalid[1].v = std::numeric_limits<double>::quiet_NaN();
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output),
                      "NaN input should fail");

        invalid = constantSpeed;
        invalid[1].a = std::numeric_limits<double>::infinity();
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output),
                      "infinite input should fail");

        invalid = {
            {0.0, 1.0, 0.0, 0.0},
            {0.8, 0.9, 0.0, 0.0},
        };
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output) &&
                          output.empty(),
                      "backward displacement should fail");
    }

    {
        const std::vector<rsim_driver::DynamicPlanSpeedPoint> numericNoise = {
            {0.0, -1e-8, -1e-8, 0.0},
            {1.0, -1e-8, -1e-8, 0.0},
        };
        rsim_driver::QpSpeedIncreasePoints densifier;
        std::vector<rsim_driver::DynamicPlanSpeedPoint> output;
        ok &= Require(densifier.increasepoints(MakeQpResult(numericNoise),
                                               &output) &&
                          !output.empty() && output.front().s == 0.0 &&
                          output.front().v == 0.0 && output.back().s == 0.0 &&
                          output.back().v == 0.0,
                      "solver-scale negative zero should be canonicalized");

        auto invalid = numericNoise;
        invalid[0].v = -1e-4;
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output) &&
                          output.empty(),
                      "meaningful reverse speed should fail");

        invalid = numericNoise;
        invalid[0].s = -1e-4;
        ok &= Require(!densifier.increasepoints(MakeQpResult(invalid), &output) &&
                          output.empty(),
                      "meaningful negative displacement should fail");
    }

    if (!ok)
        return 1;

    std::fprintf(stderr, "PASS speed_qp_increase_points smoke\n");
    return 0;
}
