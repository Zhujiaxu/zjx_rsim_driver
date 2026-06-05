/*
 * ReferenceLineGenerator — build a local smoothed reference line from global
 * world-coordinate path points.
 */
#pragma once

#include "../common/PathMatcher.hpp"
#include "ReferenceLine.hpp"
#include "ReferenceLineSmoother.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace rsim_driver
{

struct WorldPoint
{
    double x   = 0.0;
    double y   = 0.0;
};

class ReferenceLineGenerator
{
public:
    struct GenerateConfig
    {
        int forwardPoints  = 150;
        int backwardPoints = 30;
        int minPoints      = 4;

        std::size_t matchConfirmForwardPoints = 10;

        ReferenceLineSmoother::Config smoother;
    };

    ReferenceLineGenerator();
    explicit ReferenceLineGenerator(const GenerateConfig& config);

    GenerateConfig Generateconfig;

    std::unique_ptr<ReferenceLine> Generate(
        const std::vector<WorldPoint>& globalPath,
        double egoX,
        double egoY);
    std::size_t lastMatchPointIndex() const { return last_match_point_index_; }
    ReferencePoint lastProjectionPoint() const { return last_projection_point_; }

    static void RecomputeGeometry(std::vector<ReferencePoint>* points);
    static void RecomputeGeometry(std::vector<ReferencePoint>* points,
                                  const ReferencePoint& projectionPoint);

private:
    std::vector<ReferencePoint> BuildRawWindow(
        const std::vector<WorldPoint>& globalPath,
        std::size_t matchIndex);

    ReferenceLineSmoother smoother_;
    std::size_t last_match_point_index_ = 0;
    ReferencePoint last_projection_point_;
};

}  // namespace rsim_driver
