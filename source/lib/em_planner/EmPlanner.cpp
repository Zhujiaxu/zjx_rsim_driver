#include "EmPlanner.hpp"

namespace rsim_driver
{

EmPlanner::EmPlanner(const EmPlannerConfig& config)
    : EMconfig_(config),
      perception_(config.perception),
      planning_start_(config.planning_start)
{
}

const EmPlannerConfig& EmPlanner::config() const
{
    return EMconfig_;
}

void EmPlanner::SetConfig(const EmPlannerConfig& config)
{
    EMconfig_ = config;
    perception_.SetConfig(config.perception);
    planning_start_.SetConfig(config.planning_start);
}

const FrenetObstaclePerception& EmPlanner::perception() const
{
    return perception_;
}

const PlanningStart& EmPlanner::planning_start() const
{
    return planning_start_;
}

}  // namespace rsim_driver
