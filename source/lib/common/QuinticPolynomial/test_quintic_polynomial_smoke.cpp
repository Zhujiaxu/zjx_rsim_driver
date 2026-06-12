#include "QuinticPolynomial.hpp"

#include <cmath>
#include <cstdio>

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

}  // namespace

int main()
{
    rsim_driver::QuinticPolynomial1d polynomial;
    const rsim_driver::QuinticBoundary start{1.0, 0.2, -0.1};
    const rsim_driver::QuinticBoundary end{-0.4, 0.0, 0.0};
    if (!Require(rsim_driver::QuinticPolynomial1d::Create(2.5, start, end, &polynomial),
                 "valid quintic boundary should create polynomial"))
        return 1;

    if (!Require(Near(polynomial.Evaluate(0, 0.0), start.l) &&
                 Near(polynomial.Evaluate(1, 0.0), start.dl) &&
                 Near(polynomial.Evaluate(2, 0.0), start.ddl),
                 "start boundary should match"))
        return 1;

    if (!Require(Near(polynomial.Evaluate(0, 2.5), end.l) &&
                 Near(polynomial.Evaluate(1, 2.5), end.dl) &&
                 Near(polynomial.Evaluate(2, 2.5), end.ddl),
                 "end boundary should match"))
        return 1;

    if (!Require(!rsim_driver::QuinticPolynomial1d::Create(0.0, start, end, &polynomial),
                 "non-positive delta s should fail"))
        return 1;
    if (!Require(!rsim_driver::QuinticPolynomial1d::Create(2.5, start, end, nullptr),
                 "null output should fail"))
        return 1;

    std::fprintf(stderr, "PASS quintic_polynomial smoke\n");
    return 0;
}
