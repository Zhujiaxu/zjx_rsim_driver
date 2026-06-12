#include "EmPlanner.hpp"

namespace rsim_driver
{

EmPlanner::EmPlanner(const EmPlannerConfig& config)
    : EMconfig_(config),
      perception_(config.perception_config),
      planning_start_(config.planning_start_config)
{
}

const EmPlannerConfig& EmPlanner::config() const
{
    return EMconfig_;
}

void EmPlanner::SetConfig(const EmPlannerConfig& config)
{
    EMconfig_ = config;
    perception_.SetConfig(config.perception_config);
    planning_start_.SetConfig(config.planning_start_config);
}

const FrenetObstaclePerception& EmPlanner::get_perception() const
{
    return perception_;
}

const PlanningStart& EmPlanner::get_planning_start() const
{
    return planning_start_;
}

}  // namespace rsim_driver
