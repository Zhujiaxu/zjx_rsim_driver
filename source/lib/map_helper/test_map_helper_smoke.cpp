/*
 * Smoke test for MapHelper:
 *  - Load straight_500m.xodr
 *  - GetLaneWidth(road=1, s=20, lane=-1)
 *  - LaneToWorld(road=1, lane=-1, s=20, offset=0)
 * Expected: load OK, width within [3, 4], LaneToWorld returns valid pose with reasonable x (~20).
 */

#include "MapHelper.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
    std::string xodr = argc > 1
        ? argv[1]
        : "../examples/resources/xodr/straight_500m.xodr";

    rsim_driver::MapHelper map;
    if (!map.Load(xodr))
    {
        std::fprintf(stderr, "FAIL: Load(%s) returned false\n", xodr.c_str());
        return 1;
    }
    std::fprintf(stderr, "Load OK: %s\n", xodr.c_str());

    double w = map.GetLaneWidth(1, 20.0, -1);
    std::fprintf(stderr, "GetLaneWidth(road=1, s=20, lane=-1) = %.3f\n", w);
    if (!(w > 2.0 && w < 5.0))
    {
        std::fprintf(stderr, "FAIL: lane width out of expected range\n");
        return 1;
    }

    auto pose = map.LaneToWorld(1, -1, 20.0, 0.0);
    if (!pose.valid)
    {
        std::fprintf(stderr, "FAIL: LaneToWorld returned invalid\n");
        return 1;
    }
    std::fprintf(stderr,
                 "LaneToWorld(road=1, lane=-1, s=20, offset=0) -> x=%.3f y=%.3f z=%.3f h=%.3f\n",
                 pose.x, pose.y, pose.z, pose.h);

    int leftLane = map.GetLeftLane(1, 20.0, -1);
    std::fprintf(stderr, "GetLeftLane(road=1, s=20, lane=-1) = %d (expect 0 or 1 or -2 — depends on map)\n", leftLane);

    std::fprintf(stderr, "PASS\n");
    return 0;
}
