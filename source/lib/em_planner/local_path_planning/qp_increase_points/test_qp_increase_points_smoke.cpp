#include "qpincreasepoints.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
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

rsim_driver::DpPathPoint EvaluateConstantJerk(double s,
                                               double l,
                                               double lPrime,
                                               double lDoublePrime,
                                               double jerk)
{
    const double sSquared = s * s;
    return {s,
            l + lPrime * s + 0.5 * lDoublePrime * sSquared +
                jerk * sSquared * s / 6.0,
            lPrime + lDoublePrime * s + 0.5 * jerk * sSquared,
            lDoublePrime + jerk * s};
}

}  // namespace

int main()
{
    bool ok = true;

    const double jerk = 0.6;
    const rsim_driver::DpPathPoint start{0.0, 0.5, -0.2, 0.1};
    rsim_driver::DpPathPoint end = EvaluateConstantJerk(
        1.0, start.l, start.l_prime, start.l_double_prime, jerk);
    end.s = 1.0;

    // A 1 m interval has the original start, nine inserted points, and end.
    {
        rsim_driver::QpIncreasePoints densifier;
        std::vector<rsim_driver::DpPathPoint> output;
        const bool success = densifier.increasepoints({start, end}, &output);
        ok &= Require(success, "constant-jerk path should densify");
        ok &= Require(output.size() == 11U,
                      "one 1 m segment should contain eleven endpoint-inclusive samples");
        if (output.size() == 11U)
        {
            for (std::size_t i = 0; i < output.size(); ++i)
            {
                const double s = static_cast<double>(i) * 0.1;
                const rsim_driver::DpPathPoint expected = EvaluateConstantJerk(
                    s,
                    start.l,
                    start.l_prime,
                    start.l_double_prime,
                    jerk);
                ok &= Require(Near(output[i].s, s), "sample s should advance by 0.1 m");
                ok &= Require(Near(output[i].l, expected.l),
                              "sample l should follow cubic Taylor expansion");
                ok &= Require(Near(output[i].l_prime, expected.l_prime),
                              "sample l_prime should follow cubic Taylor expansion");
                ok &= Require(Near(output[i].l_double_prime,
                                   expected.l_double_prime),
                              "sample l_double_prime should follow cubic Taylor expansion");
            }
        }
    }

    // Invalid input must fail without preserving stale output.
    {
        rsim_driver::QpIncreasePoints densifier;
        std::vector<rsim_driver::DpPathPoint> output{{99.0, 0.0, 0.0, 0.0}};
        const bool success = densifier.increasepoints({start}, &output);
        ok &= Require(!success, "single-point path should fail");
        ok &= Require(output.empty(), "failed densification should clear output");
    }

    {
        rsim_driver::QpIncreasePoints densifier({0});
        std::vector<rsim_driver::DpPathPoint> output;
        const bool success = densifier.increasepoints({start, end}, &output);
        ok &= Require(!success, "zero subdivisions should fail");
        ok &= Require(output.empty(), "invalid config should leave output empty");
    }

    {
        rsim_driver::QpIncreasePoints densifier;
        std::vector<rsim_driver::DpPathPoint> output;
        rsim_driver::DpPathPoint duplicate = end;
        duplicate.s = start.s;
        const bool success = densifier.increasepoints({start, duplicate}, &output);
        ok &= Require(!success, "non-increasing s should fail");
        ok &= Require(output.empty(), "non-increasing s should clear output");
    }

    {
        rsim_driver::QpIncreasePoints densifier;
        std::vector<rsim_driver::DpPathPoint> output;
        rsim_driver::DpPathPoint invalid = end;
        invalid.l = std::numeric_limits<double>::quiet_NaN();
        const bool success = densifier.increasepoints({start, invalid}, &output);
        ok &= Require(!success, "non-finite input should fail");
        ok &= Require(output.empty(), "non-finite input should clear output");
    }

    {
        rsim_driver::QpIncreasePoints densifier;
        std::vector<rsim_driver::DpPathPoint> inconsistent = {start, end};
        inconsistent.back().l_prime += 0.1;
        std::vector<rsim_driver::DpPathPoint> output;
        const bool success = densifier.increasepoints(inconsistent, &output);
        ok &= Require(!success, "Taylor-inconsistent QP output should fail");
        ok &= Require(output.empty(), "inconsistent input should clear output");
    }

    if (!ok)
        return 1;

    std::fprintf(stderr, "PASS qp_increase_points smoke\n");
    return 0;
}
