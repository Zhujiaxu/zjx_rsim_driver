#include "dpincreasepoints.hpp"

#include <cmath>
#include <cstdio>

namespace
{

bool Require(bool condition, const char *message)
{
    if (!condition)
        std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

}  // namespace

int main()
{
    rsim_driver::DpPlannerResult input;
    input.dpsuccess = true;
    input.path = {{0.0, 0.0, 0.0, 0.0},
                  {1.0, 1.0, 0.0, 0.0}};
    rsim_driver::DPIncreasePoints densifier({4});
    rsim_driver::DpPlannerResult output;
    if (!Require(densifier.increasepoints(&input, &output) &&
                     output.path.size() == 5 && output.dpsuccess &&
                     std::fabs(output.path.front().s) < 1e-9 &&
                     std::fabs(output.path.back().s - 1.0) < 1e-9,
                 "DP path should densify by the configured subdivision count"))
        return 1;

    input.path[1].s = input.path[0].s;
    if (!Require(!densifier.increasepoints(&input, &output) &&
                     output.path.empty(),
                 "non-increasing input should fail and clear output"))
        return 1;
    if (!Require(!densifier.increasepoints(nullptr, &output) &&
                     !densifier.increasepoints(&input, nullptr),
                 "null arguments should fail"))
        return 1;

    std::fprintf(stderr, "PASS dp_increase_points smoke\n");
    return 0;
}
