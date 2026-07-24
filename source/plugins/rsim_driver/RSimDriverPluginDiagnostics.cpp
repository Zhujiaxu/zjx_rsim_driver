#include "RSimDriverPlugin.hpp"

#include <cstdio>
#include <limits>

namespace rsim_driver::plugin_internal
{

const char *RSimDriverPlugin::PlanningStartSourceName(
    PlanningStartSource source) const
{
    switch (source)
    {
    case PlanningStartSource::KinematicExtrapolation:
        return "KinematicExtrapolation";
    case PlanningStartSource::PreviousTrajectory:
        return "PreviousTrajectory";
    }
    return "Unknown";
}

const char *RSimDriverPlugin::PlannerFailureStage(
    const EmPlannerResult &result) const
{
    if (!result.static_perception_success)
        return "static_perception";
    if (!result.virtual_perception_success)
        return "virtual_perception";
    if (!result.planning_start_success)
        return "planning_start";
    if (!result.frenet_start_success)
        return "frenet_start";
    if (!result.dp_success)
        return "path_dp";
    if (!result.dp_increase_points_success)
        return "path_dp_increase";
    if (!result.drivable_area_success)
        return "path_drivable_area";
    if (!result.qp_success)
        return "path_qp";
    if (!result.qp_increase_points_success)
        return "path_qp_increase";
    if (!result.qp_increase_points_to_frenet)
        return "path_to_cartesian";
    if (result.speed_reference_line.empty())
        return "speed_reference_line";
    if (!result.dynamic_perception_success)
        return "dynamic_perception";
    if (!result.speed_dp_success)
        return "speed_boundary_or_dp";
    if (!result.st_drivable_area_success)
        return "speed_drivable_area";
    if (!result.speed_qp_success)
        return "speed_qp";
    if (!result.speed_qp_increase_points_success)
        return "speed_qp_increase";
    if (!result.trajectory_success)
        return "trajectory";
    return "unknown";
}

void RSimDriverPlugin::ReportPlannerStageStatus(
    const rsim_plugin::TickContext &ctx,
    const EmPlannerResult &result) const
{
    std::fprintf(
        stderr,
        "[RSimDriver] EM planner stages frame=%llu time=%.6f "
        "first_failed=%s static=%d virtual=%d start=%d frenet=%d "
        "dp=%d dp_increase=%d drivable=%d qp=%d qp_increase=%d "
        "path_cartesian=%d dynamic=%d speed_ref=%zu speed_dp=%d "
        "st_area=%d speed_qp=%d speed_dense=%d trajectory=%d "
        "static_obs=%zu virtual_obs=%zu dynamic_obs=%zu "
        "virtual_seeds=%zu dp_points=%zu qp_points=%zu "
        "cartesian_points=%zu speed_dp_points=%zu speed_qp_points=%zu "
        "dense_points=%zu trajectory_points=%zu previous_points=%zu\n",
        static_cast<unsigned long long>(ctx.frame_id),
        ctx.sim_time,
        PlannerFailureStage(result),
        result.static_perception_success ? 1 : 0,
        result.virtual_perception_success ? 1 : 0,
        result.planning_start_success ? 1 : 0,
        result.frenet_start_success ? 1 : 0,
        result.dp_success ? 1 : 0,
        result.dp_increase_points_success ? 1 : 0,
        result.drivable_area_success ? 1 : 0,
        result.qp_success ? 1 : 0,
        result.qp_increase_points_success ? 1 : 0,
        result.qp_increase_points_to_frenet ? 1 : 0,
        result.dynamic_perception_success ? 1 : 0,
        result.speed_reference_line.size(),
        result.speed_dp_success ? 1 : 0,
        result.st_drivable_area_success ? 1 : 0,
        result.speed_qp_success ? 1 : 0,
        result.speed_qp_increase_points_success ? 1 : 0,
        result.trajectory_success ? 1 : 0,
        result.static_perception_result.staticobstacles.size(),
        result.virtual_perception_result.virtualstaticobstacles.size(),
        result.dynamic_perception_result.dynamicobstacles.size(),
        result.virtual_obstacle_seeds.size(),
        result.dp_result.path.size(),
        result.qp_increase_points_result.localfrenetpath.size(),
        result.localcartesianpath.size(),
        result.speed_dp_result.stpoints.size(),
        result.speed_qp_result.stpoints.size(),
        result.speed_increasepoints_line.size(),
        result.trajectory.size(),
        previous_trajectory_.size());
}

void RSimDriverPlugin::ReportPlanningFailure(
    const rsim_plugin::TickContext &ctx,
    const rsim_plugin::ActorState &ego,
    const char *reason) const
{
    PluginLog("[RSimDriver] ERROR: %s, stop ego id=%d frame=%llu "
             "time=%.6f ego=(%.3f, %.3f, h=%.3f)\n",
             reason,
             ego.id,
             static_cast<unsigned long long>(ctx.frame_id),
             ctx.sim_time,
             ego.x,
             ego.y,
             ego.h);
}

void RSimDriverPlugin::OpenReferenceLineDebugCsv()
{
    if (reference_line_csv_fp_ != nullptr)
    {
        std::fclose(reference_line_csv_fp_);
        reference_line_csv_fp_ = nullptr;
    }
    if (reference_line_csv_path_.empty())
        return;

    reference_line_csv_fp_ =
        std::fopen(reference_line_csv_path_.c_str(), "w");
    if (reference_line_csv_fp_ == nullptr)
    {
        PluginLog("[RSimDriver] WARNING: cannot write reference line CSV: %s\n",
                 reference_line_csv_path_.c_str());
        reference_line_csv_path_.clear();
        return;
    }

    std::fprintf(reference_line_csv_fp_,
                 "frame_id,sim_time,ego_x,ego_y,global_match_idx,"
                 "projection_x,projection_y,projection_hdg,"
                 "target_idx,point_idx,ref_s,ref_x,ref_y,ref_hdg,"
                 "target_x,target_y\n");
    std::fflush(reference_line_csv_fp_);
}

void RSimDriverPlugin::WriteReferenceLineDebugCsv(
    const rsim_plugin::TickContext &ctx,
    const rsim_plugin::ActorState &ego,
    std::size_t targetIndex,
    const CartesianPathPoint &target)
{
    if (reference_line_csv_fp_ == nullptr || reference_line_ == nullptr)
        return;

    const ReferencePoint projectionPoint =
        reference_line_generator_.curProjectionPoint();
    for (std::size_t i = 0; i < reference_line_->points.size(); ++i)
    {
        const ReferencePoint &point = reference_line_->points[i];
        std::fprintf(reference_line_csv_fp_,
                     "%llu,%.9f,%.9f,%.9f,%zu,"
                     "%.9f,%.9f,%.9f,"
                     "%zu,%zu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                     static_cast<unsigned long long>(ctx.frame_id),
                     ctx.sim_time,
                     ego.x,
                     ego.y,
                     reference_line_generator_.lastMatchPointIndex(),
                     projectionPoint.x,
                     projectionPoint.y,
                     projectionPoint.hdg,
                     targetIndex,
                     i,
                     point.s,
                     point.x,
                     point.y,
                     point.hdg,
                     target.x,
                     target.y);
    }
    std::fflush(reference_line_csv_fp_);
}

void RSimDriverPlugin::OpenPlanningStartSlDebugCsv()
{
    if (planning_start_sl_csv_fp_ != nullptr)
    {
        std::fclose(planning_start_sl_csv_fp_);
        planning_start_sl_csv_fp_ = nullptr;
    }
    if (planning_start_sl_csv_path_.empty())
        return;

    planning_start_sl_csv_fp_ =
        std::fopen(planning_start_sl_csv_path_.c_str(), "w");
    if (planning_start_sl_csv_fp_ == nullptr)
    {
        PluginLog("[RSimDriver] WARNING: cannot write planning-start CSV: %s\n",
                 planning_start_sl_csv_path_.c_str());
        planning_start_sl_csv_path_.clear();
        return;
    }

    std::fprintf(planning_start_sl_csv_fp_,
                 "frame_id,sim_time,time_step,"
                 "ego_x,ego_y,ego_h,ego_speed,ego_acc_x,ego_acc_y,ego_accel,"
                 "start_x,start_y,start_heading,start_speed,start_accel,start_time,"
                 "start_source,match_distance,start_curvature,"
                 "sl_success,s,l,l_prime,l_double_prime,"
                 "previous_trajectory_size,stitching_trajectory_size\n");
    std::fflush(planning_start_sl_csv_fp_);
}

void RSimDriverPlugin::WritePlanningStartSlDebugCsv(
    const rsim_plugin::TickContext &ctx,
    const rsim_plugin::ActorState &ego,
    const PlanningStartResult &startResult,
    const StartPointFrenetState &frenet,
    bool slSuccess)
{
    if (planning_start_sl_csv_fp_ == nullptr)
        return;

    const PlanningStartPoint &start = startResult.start_point;
    const auto &startPt = start.startpointbasis;
    const double egoAccel = ego.acc_x;
    std::fprintf(planning_start_sl_csv_fp_,
                 "%llu,%.9f,%.9f,"
                 "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                 "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                 "%s,%.9f,%.9f,"
                 "%d,%.9f,%.9f,%.9f,%.9f,"
                 "%zu,%zu\n",
                 static_cast<unsigned long long>(ctx.frame_id),
                 ctx.sim_time,
                 ctx.time_step,
                 ego.x,
                 ego.y,
                 ego.h,
                 ego.speed,
                 ego.acc_x,
                 ego.acc_y,
                 egoAccel,
                 startPt.x,
                 startPt.y,
                 startPt.heading,
                 startPt.speed,
                 startPt.accel,
                 startPt.time,
                 PlanningStartSourceName(start.source),
                 start.matchDistance,
                 startResult.start_curvature,
                 slSuccess ? 1 : 0,
                 frenet.s,
                 frenet.l,
                 frenet.l_prime,
                 frenet.l_double_prime,
                 previous_trajectory_.size(),
                 startResult.stitching_trajectory.size());
    std::fflush(planning_start_sl_csv_fp_);
}

void RSimDriverPlugin::OpenEgoTrajectoryCsv()
{
    if (ego_trajectory_csv_fp_ != nullptr)
    {
        std::fclose(ego_trajectory_csv_fp_);
        ego_trajectory_csv_fp_ = nullptr;
    }
    if (ego_trajectory_csv_path_.empty())
        return;

    ego_trajectory_csv_fp_ = std::fopen(ego_trajectory_csv_path_.c_str(), "w");
    if (ego_trajectory_csv_fp_ == nullptr)
    {
        PluginLog("[RSimDriver] WARNING: cannot write ego trajectory CSV: %s\n",
                 ego_trajectory_csv_path_.c_str());
        ego_trajectory_csv_path_.clear();
        return;
    }

    std::fprintf(ego_trajectory_csv_fp_,
                 "frame_id,sim_time,time_step,ego_x,ego_y,ego_h,"
                 "target_idx,point_idx,s,l,l_prime,l_double_prime,"
                 "x,y,theta,k,v,a,time,is_target\n");
    std::fflush(ego_trajectory_csv_fp_);
}

void RSimDriverPlugin::WriteEgoTrajectoryCsv(
    const rsim_plugin::TickContext &ctx,
    const rsim_plugin::ActorState &ego,
    std::size_t targetIndex,
    const std::vector<DpPathPoint> &localFrenetPath,
    const std::vector<PlanningTrajectoryPoint> &path)
{
    if (ego_trajectory_csv_fp_ == nullptr)
        return;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        const DpPathPoint *frenetPoint =
            i < localFrenetPath.size() ? &localFrenetPath[i] : nullptr;
        const PlanningTrajectoryPoint &point = path[i];
        std::fprintf(ego_trajectory_csv_fp_,
                     "%llu,%.9f,%.9f,%.9f,%.9f,%.9f,"
                     "%zu,%zu,%.9f,%.9f,%.9f,%.9f,"
                     "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d\n",
                     static_cast<unsigned long long>(ctx.frame_id),
                     ctx.sim_time,
                     ctx.time_step,
                     ego.x,
                     ego.y,
                     ego.h,
                     targetIndex,
                     i,
                     frenetPoint != nullptr ? frenetPoint->s : nan,
                     frenetPoint != nullptr ? frenetPoint->l : nan,
                     frenetPoint != nullptr ? frenetPoint->l_prime : nan,
                     frenetPoint != nullptr ? frenetPoint->l_double_prime : nan,
                     point.x,
                     point.y,
                     point.heading,
                     point.curvature,
                     point.speed,
                     point.accel,
                     point.time,
                     i == targetIndex ? 1 : 0);
    }
    std::fflush(ego_trajectory_csv_fp_);
}

} // namespace rsim_driver::plugin_internal
