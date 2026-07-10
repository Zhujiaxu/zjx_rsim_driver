#include "increasepoints.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

bool Near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

rsim_driver::DpPlannerResult MakePath(const std::vector<double> &s_values,
                                      double l = 0.0)
{
    rsim_driver::DpPlannerResult result;
    for (double s : s_values)
    {
        result.path.push_back({s, l, 0.0, 0.0});
    }
    result.dpsuccess = true;
    result.total_cost = 10.0;
    return result;
}

} // namespace

int main()
{
    bool ok = true;

    // --- basic: count=2, 3-point path ---
    {
        rsim_driver::IncreasePoints ip({2});
        auto input = MakePath({0.0, 0.5, 1.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(success, "basic: should succeed");
        ok &= Require(output.path.size() == 5, "basic: output size == 5");
        if (output.path.size() == 5)
        {
            ok &= Require(Near(output.path[0].s, 0.0), "basic: s[0] == 0.0");
            ok &= Require(Near(output.path[1].s, 0.25), "basic: s[1] == 0.25");
            ok &= Require(Near(output.path[2].s, 0.5), "basic: s[2] == 0.5");
            ok &= Require(Near(output.path[3].s, 0.75), "basic: s[3] == 0.75");
            ok &= Require(Near(output.path[4].s, 1.0), "basic: s[4] == 1.0");
        }
        ok &= Require(output.dpsuccess, "basic: dpsuccess propagated");
        ok &= Require(Near(output.total_cost, 10.0), "basic: total_cost propagated");
    }

    // --- count=1: output equals original path ---
    {
        rsim_driver::IncreasePoints ip({1});
        auto input = MakePath({0.0, 0.5, 1.0}, 1.5);
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(success, "count=1: should succeed");
        ok &= Require(output.path.size() == 3, "count=1: output size == 3");
        if (output.path.size() == 3)
        {
            ok &= Require(Near(output.path[0].s, 0.0) && Near(output.path[0].l, 1.5),
                          "count=1: point[0] matches");
            ok &= Require(Near(output.path[1].s, 0.5) && Near(output.path[1].l, 1.5),
                          "count=1: point[1] matches");
            ok &= Require(Near(output.path[2].s, 1.0) && Near(output.path[2].l, 1.5),
                          "count=1: point[2] matches");
        }
    }

    // --- invalid count: 0 ---
    {
        rsim_driver::IncreasePoints ip({0});
        auto input = MakePath({0.0, 1.0});
        rsim_driver::DpPlannerResult output;
        output.path.push_back({999.0, 999.0, 0.0, 0.0}); // pre-populate
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "count=0: should fail");
        ok &= Require(output.path.empty(), "count=0: output should be cleared");
    }

    // --- invalid count: -1 ---
    {
        rsim_driver::IncreasePoints ip({-1});
        auto input = MakePath({0.0, 1.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "count=-1: should fail");
    }

    // --- empty input path ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "empty path: should fail");
    }

    // --- single-point input path ---
    {
        rsim_driver::IncreasePoints ip({2});
        auto input = MakePath({0.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "single point: should fail");
    }

    // --- null input ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(nullptr, &output);
        ok &= Require(!success, "null result: should fail");
    }

    // --- null output ---
    {
        rsim_driver::IncreasePoints ip({2});
        auto input = MakePath({0.0, 1.0});
        bool success = ip.increasepoints(&input, nullptr);
        ok &= Require(!success, "null newresult: should fail");
    }

    // --- non-increasing s (duplicate) ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({1.0, 0.0, 0.0, 0.0});
        input.path.push_back({1.0, 0.0, 0.0, 0.0}); // duplicate s
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "duplicate s: should fail");
    }

    // --- non-increasing s (decreasing) ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({2.0, 0.0, 0.0, 0.0});
        input.path.push_back({1.0, 0.0, 0.0, 0.0}); // decreasing s
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "decreasing s: should fail");
    }

    // --- NaN s value ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "NaN s: should fail");
    }

    // --- Inf l value ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({1.0, std::numeric_limits<double>::infinity(), 0.0, 0.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "Inf l: should fail");
    }

    // --- delta_s too small ---
    {
        rsim_driver::IncreasePoints ip({2});
        rsim_driver::DpPlannerResult input;
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({1e-12, 0.0, 0.0, 0.0}); // delta_s < 1e-9
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(!success, "tiny delta_s: should fail");
    }

    // --- count=3, 3-point curved path ---
    {
        rsim_driver::IncreasePoints ip({3});
        rsim_driver::DpPlannerResult input;
        // s=0 l=0, s=3 l=2, s=6 l=1
        input.path.push_back({0.0, 0.0, 0.0, 0.0});
        input.path.push_back({3.0, 2.0, 0.5, -0.1});
        input.path.push_back({6.0, 1.0, -0.2, 0.0});
        input.dpsuccess = true;
        input.total_cost = 5.0;
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(success, "count=3 curved: should succeed");
        // size = 1 (first point) + 2 segments * 3 = 7
        ok &= Require(output.path.size() == 7, "count=3 curved: size == 7");
        ok &= Require(output.dpsuccess, "count=3 curved: dpsuccess");
        ok &= Require(Near(output.total_cost, 5.0), "count=3 curved: total_cost");

        // Verify s is strictly increasing
        if (output.path.size() >= 2)
        {
            bool s_increasing = true;
            for (size_t i = 1; i < output.path.size(); ++i)
            {
                if (output.path[i].s <= output.path[i - 1].s)
                {
                    s_increasing = false;
                    break;
                }
            }
            ok &= Require(s_increasing, "count=3 curved: s strictly increasing");
        }

        // Verify first and last points match original endpoints
        ok &= Require(Near(output.path.front().s, 0.0) && Near(output.path.front().l, 0.0),
                      "count=3 curved: first point matches");
        ok &= Require(Near(output.path.back().s, 6.0) && Near(output.path.back().l, 1.0),
                      "count=3 curved: last point matches");
    }

    // --- count=4, 2-point path (single segment) ---
    {
        rsim_driver::IncreasePoints ip({4});
        auto input = MakePath({0.0, 2.0}, 1.0);
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(success, "count=4 2pt: should succeed");
        // size = 1 (first point) + 4 (one segment * 4) = 5
        ok &= Require(output.path.size() == 5, "count=4 2pt: size == 5");
        ok &= Require(Near(output.path.front().s, 0.0), "count=4 2pt: s[0] == 0.0");
        ok &= Require(Near(output.path.back().s, 2.0), "count=4 2pt: last s == 2.0");
    }

    // --- SetConfig + config() ---
    {
        rsim_driver::IncreasePoints ip({2});
        ok &= Require(ip.config().count == 2, "SetConfig: initial count == 2");

        ip.SetConfig({5});
        ok &= Require(ip.config().count == 5, "SetConfig: count updated to 5");

        auto input = MakePath({0.0, 1.0});
        rsim_driver::DpPlannerResult output;
        bool success = ip.increasepoints(&input, &output);
        ok &= Require(success, "SetConfig: count=5 should succeed");
        // size = 1 + 1 * 5 = 6
        ok &= Require(output.path.size() == 6, "SetConfig: size with count=5 == 6");
    }

    if (ok)
    {
        std::fprintf(stdout, "PASS\n");
        return 0;
    }
    return 1;
}
