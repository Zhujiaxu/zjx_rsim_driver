#include "EmPlanner.hpp"

namespace rsim_driver
{

EmPlanner::EmPlanner(const EmPlannerConfig& config)
    : EMconfig_(config),
      perception_(config.perception_config),
      planning_start_(config.planning_start_config),
      dp_planner_(config.dp_config),
      drivable_area_builder_(config.drivable_area_config),
      qp_path_optimizer_(config.qp_config)
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
    dp_planner_.SetConfig(config.dp_config);
    drivable_area_builder_.SetConfig(config.drivable_area_config);
    qp_path_optimizer_.SetConfig(config.qp_config);
}

const FrenetObstaclePerception& EmPlanner::get_perception() const
{
    return perception_;
}

const PlanningStart& EmPlanner::get_planning_start() const
{
    return planning_start_;
}

bool EmPlanner::RunDynamicProgramming(
    const CartesianFrenetState& start,
    const std::vector<StaticFrenetObstacle>& obstacles,
    DpPlannerResult* result) const
{
    return dp_planner_.Plan(start, obstacles, result);
}

bool EmPlanner::BuildDrivableArea(
    const std::vector<DpPathPoint>& coarsePath,
    const std::vector<StaticFrenetObstacle>& obstacles,
    DrivableArea* result) const
{
    return drivable_area_builder_.Build(coarsePath, obstacles, result);
}

bool EmPlanner::RunQuadraticProgramming(
    const CartesianFrenetState& start,
    const std::vector<DpPathPoint>& coarsePath,
    const DrivableArea& drivableArea,
    const std::vector<StaticFrenetObstacle>& obstacles,
    QpPathResult* result) const
{
    return qp_path_optimizer_.Optimize(start,
                                       coarsePath,
                                       drivableArea,
                                       obstacles,
                                       result);
}

}  // namespace rsim_driver
