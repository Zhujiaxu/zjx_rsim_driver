#include "localpathreferline.hpp"

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

}  // namespace

int main()
{
    std::vector<rsim_driver::CartesianPathPoint> cartesian(3);
    cartesian[0].x = 0.0;
    cartesian[0].kappa = 0.0;
    cartesian[1].x = 3.0;
    cartesian[1].y = 4.0;
    cartesian[1].kappa = 0.1;
    cartesian[2].x = 6.0;
    cartesian[2].y = 8.0;
    cartesian[2].kappa = 0.2;

    rsim_driver::localreferencelinepath output;
    if (!Require(rsim_driver::LocalCartesianPathToReferenceLinePath(
                     cartesian, &output) && output.size() == 3 &&
                     std::fabs(output[1].s - 5.0) < 1e-9 &&
                     std::fabs(output[2].s - 10.0) < 1e-9 &&
                     std::fabs(output[0].dk - 0.02) < 1e-9,
                 "Cartesian path should convert to a strict arc-length grid"))
        return 1;

    auto invalid = cartesian;
    invalid[1] = invalid[0];
    if (!Require(!rsim_driver::LocalCartesianPathToReferenceLinePath(
                     invalid, &output) && output.empty(),
                 "duplicate Cartesian points should fail"))
        return 1;
    invalid = cartesian;
    invalid[1].x = std::numeric_limits<double>::quiet_NaN();
    if (!Require(!rsim_driver::LocalCartesianPathToReferenceLinePath(
                     invalid, &output) &&
                     !rsim_driver::LocalCartesianPathToReferenceLinePath(
                         cartesian, nullptr),
                 "non-finite input and null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS local_reference_line smoke\n");
    return 0;
}
