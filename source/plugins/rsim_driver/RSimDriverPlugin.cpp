#include "RSimDriverPlugin.hpp"
#include "TrajectorySampling.hpp"

#include <cmath>
#include <cstdio>
#include <exception>
#include <utility>

namespace rsim_driver::plugin_internal
{

namespace
{

constexpr double kTrajectoryUpdateLookahead = 0.05;

} // namespace

RSimDriverPlugin::~RSimDriverPlugin()
{
    if (reference_line_csv_fp_ != nullptr)
        std::fclose(reference_line_csv_fp_);
    if (planning_start_sl_csv_fp_ != nullptr)
        std::fclose(planning_start_sl_csv_fp_);
    if (ego_trajectory_csv_fp_ != nullptr)
        std::fclose(ego_trajectory_csv_fp_);
    if (log_fp_ != nullptr)
    {
        SetPluginLogFile(nullptr);
        std::fclose(log_fp_);
        log_fp_ = nullptr;
    }
}

void RSimDriverPlugin::Init(
    const std::map<std::string, std::string> &properties,
    const std::vector<int32_t> &controlledActorIds)
{
    initialization_ok_ = true;
    controlled_ids_ = controlledActorIds;

    const auto getString = [&](const char *key, const char *defaultValue)
    {
        const auto it = properties.find(key);
        return it != properties.end() ? it->second : defaultValue;
    };
    const auto getDouble = [&](const char *key,
                               double defaultValue,
                               double *value) -> bool
    {
        if (value == nullptr)
            return false;
        const auto it = properties.find(key);
        if (it == properties.end())
        {
            *value = defaultValue;
            return true;
        }
        try
        {
            std::size_t parsed = 0;
            const double parsedValue = std::stod(it->second, &parsed);
            if (parsed != it->second.size() || !std::isfinite(parsedValue))
                return false;
            *value = parsedValue;
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    };

    const std::string xodrPath = getString("xodrPath", "");
    route_xosc_path_ = getString("routeXoscPath", "");
    route_csv_path_ = getString("routeCsvPath", "");
    reference_line_csv_path_ = getString("referenceLineCsvPath", "");
    obstacle_csv_path_ = getString("obstacleCsvPath", "");
    planning_start_sl_csv_path_ = getString("planningStartSlCsvPath", "");
    ego_trajectory_csv_path_ = getString("egoTrajectoryCsvPath", "");
    entity_name_ = getString("entityName", "ego");
    log_file_path_ = getString("logFilePath", "");

    if (!log_file_path_.empty())
    {
        log_fp_ = std::fopen(log_file_path_.c_str(), "w");
        if (log_fp_ != nullptr)
        {
            SetPluginLogFile(log_fp_);
            PluginLog("[RSimDriver] log file opened: %s\n", log_file_path_.c_str());
        }
    }

    if (!getDouble("setSpeed", 13.0, &set_speed_) || set_speed_ < 0.0)
    {
        PluginLog("[RSimDriver] FATAL: invalid setSpeed property\n");
        initialization_ok_ = false;
        return;
    }

    EmPlannerConfig plannerConfig = em_planner_.config();
    plannerConfig.speed_dp_config.reference_speed = set_speed_;
    plannerConfig.speed_qp_config.reference_speed = set_speed_;
    em_planner_.SetConfig(plannerConfig);

    OpenReferenceLineDebugCsv();
    OpenPlanningStartSlDebugCsv();
    OpenEgoTrajectoryCsv();
    obstacle_csv_writer_.Open(obstacle_csv_path_);

    if (xodrPath.empty() || !map_.Load(xodrPath))
    {
        PluginLog("[RSimDriver] FATAL: xodrPath property is missing or map "
                 "loading failed ('%s')\n",
                 xodrPath.c_str());
        map_loaded_ = false;
    }
    else
    {
        map_loaded_ = true;
    }

    if (map_loaded_ && !InstallGlobalPathFromXosc(route_xosc_path_))
    {
        PluginLog("[RSimDriver] FATAL: failed to install route from XOSC\n");
        initialization_ok_ = false;
    }
}

std::vector<rsim_plugin::ActorUpdate> RSimDriverPlugin::Step(
    const rsim_plugin::TickContext &ctx)
{
    std::vector<rsim_plugin::ActorUpdate> updates;
    if (!initialization_ok_ || controlled_ids_.empty())
        return updates;

    const rsim_plugin::ActorState *ego = FindControlledActor(ctx);
    if (ego == nullptr)
        return updates;

    if (!map_loaded_)
    {
        ReportPlanningFailure(ctx, *ego, "map is not loaded");
        updates.push_back(BuildStopActorUpdate(*ego));
        return updates;
    }

    obstacle_csv_writer_.WriteFrame(ctx.frame_id,
                                    ctx.sim_time,
                                    *ego,
                                    ctx.actors,
                                    controlled_ids_);

    LatchInitialState(*ego);
    UpdateReferenceLine(*ego);
    if (reference_line_ == nullptr || reference_line_->points.empty())
    {
        ReportPlanningFailure(ctx, *ego, "reference line is empty");
        updates.push_back(BuildStopActorUpdate(*ego));
        return updates;
    }

    EmPlannerResult plannerResult;
    if (!RunEmPlannerDetailed(ctx, *ego, &plannerResult))
    {
        if (plannerResult.planning_start_success)
        {
            WritePlanningStartSlDebugCsv(ctx,
                                         *ego,
                                         plannerResult.planning_start_result,
                                         plannerResult.frenet_start_result,
                                         plannerResult.frenet_start_success);
        }
        ReportPlannerStageStatus(ctx, plannerResult);
        ReportPlanningFailure(ctx, *ego, "EM planner failed");
        updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
        return updates;
    }

    WritePlanningStartSlDebugCsv(ctx,
                                 *ego,
                                 plannerResult.planning_start_result,
                                 plannerResult.frenet_start_result,
                                 plannerResult.frenet_start_success);
    previous_trajectory_ = plannerResult.trajectory;

    if (plannerResult.trajectory.empty())
    {
        ReportPlannerStageStatus(ctx, plannerResult);
        ReportPlanningFailure(ctx, *ego, "EM planner trajectory is empty");
        updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
        return updates;
    }

    std::size_t targetIndex = 0;
    PlanningTrajectoryPoint target;
    const double targetTime = ctx.sim_time + kTrajectoryUpdateLookahead;
    if (!FindTrajectoryPointAtTime(plannerResult.trajectory,
                                   targetTime,
                                   &target,
                                   &targetIndex))
    {
        ReportPlannerStageStatus(ctx, plannerResult);
        ReportPlanningFailure(
            ctx, *ego, "EM planner trajectory does not cover update time");
        updates.push_back(BuildControlledStopActorUpdate(*ego, ctx.time_step));
        return updates;
    }

    WriteEgoTrajectoryCsv(
        ctx,
        *ego,
        targetIndex,
        plannerResult.qp_increase_points_result.localfrenetpath,
        plannerResult.trajectory);
    updates.push_back(BuildActorUpdateFromTrajectoryPoint(*ego, target));

    CartesianPathPoint debugTarget;
    debugTarget.x = target.x;
    debugTarget.y = target.y;
    debugTarget.heading = target.heading;
    debugTarget.kappa = target.curvature;
    WriteReferenceLineDebugCsv(ctx, *ego, targetIndex, debugTarget);
    return updates;
}

} // namespace rsim_driver::plugin_internal

extern "C" rsim_plugin::IPluginController *CreateController(const char *name)
{
    (void)name;
    return new rsim_driver::plugin_internal::RSimDriverPlugin();
}

extern "C" void DestroyController(rsim_plugin::IPluginController *controller)
{
    delete controller;
}
