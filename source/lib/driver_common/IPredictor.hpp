/*
 * IPredictor — obstacle prediction interface.
 *
 * Default implementation: ConstantVelocityPredictor (s(t) = s0 + speed * t).
 * Planners call Predict() to get obstacle states at future query times.
 */
#pragma once

#include "DriverTypes.hpp"
#include <vector>

namespace rsim_driver
{

struct PredictedObstacle
{
    double s     = 0.0;
    double d     = 0.0;
    double speed = 0.0;
    double t     = 0.0;
};

class IPredictor
{
public:
    virtual ~IPredictor() = default;

    // For a given obstacle and set of query times, return predicted states.
    virtual std::vector<PredictedObstacle> Predict(
        const Obstacle& obs,
        const std::vector<double>& times) const = 0;
};

class ConstantVelocityPredictor : public IPredictor
{
public:
    std::vector<PredictedObstacle> Predict(
        const Obstacle& obs,
        const std::vector<double>& times) const override
    {
        std::vector<PredictedObstacle> result;
        result.reserve(times.size());
        for (double t : times)
        {
            PredictedObstacle po;
            po.t     = t;
            po.s     = obs.s + obs.speed * t;
            po.d     = obs.d;
            po.speed = obs.speed;
            result.push_back(po);
        }
        return result;
    }
};

}  // namespace rsim_driver
