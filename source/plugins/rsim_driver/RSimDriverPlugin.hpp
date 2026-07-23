#pragma once

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include "EmPlanner.hpp"
#include "GlobalPathGenerator.hpp"
#include "MapHelper.hpp"
#include "ObstacleToCsv.hpp"
#include "ReferenceLineGenerator.hpp"

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rsim_driver::plugin_internal
{

class RSimDriverPlugin final : public rsim_plugin::IPluginController
{
public:
    ~RSimDriverPlugin() override;

    void Init(const std::map<std::string, std::string> &properties,
              const std::vector<int32_t> &controlledActorIds) override;

    std::vector<rsim_plugin::ActorUpdate> Step(
        const rsim_plugin::TickContext &ctx) override;

private:
    bool RunEmPlannerDetailed(const rsim_plugin::TickContext &ctx,
                              const rsim_plugin::ActorState &ego,
                              EmPlannerResult *result);

    bool InstallGlobalPathFromXosc(const std::string &xoscPath);
    const rsim_plugin::ActorState *FindControlledActor(
        const rsim_plugin::TickContext &ctx) const;
    rsim_plugin::ActorUpdate BuildStopActorUpdate(
        const rsim_plugin::ActorState &ego) const;
    rsim_plugin::ActorUpdate BuildControlledStopActorUpdate(
        const rsim_plugin::ActorState &ego,
        double dt) const;
    rsim_plugin::ActorUpdate BuildActorUpdateFromTrajectoryPoint(
        const rsim_plugin::ActorState &ego,
        const PlanningTrajectoryPoint &target) const;
    void LatchInitialState(const rsim_plugin::ActorState &ego);
    void UpdateReferenceLine(const rsim_plugin::ActorState &ego);

    const char *PlanningStartSourceName(PlanningStartSource source) const;
    const char *PlannerFailureStage(const EmPlannerResult &result) const;
    void ReportPlannerStageStatus(const rsim_plugin::TickContext &ctx,
                                  const EmPlannerResult &result) const;
    void ReportPlanningFailure(const rsim_plugin::TickContext &ctx,
                               const rsim_plugin::ActorState &ego,
                               const char *reason) const;

    void OpenReferenceLineDebugCsv();
    void WriteReferenceLineDebugCsv(const rsim_plugin::TickContext &ctx,
                                    const rsim_plugin::ActorState &ego,
                                    std::size_t targetIndex,
                                    const CartesianPathPoint &target);
    void OpenPlanningStartSlDebugCsv();
    void WritePlanningStartSlDebugCsv(
        const rsim_plugin::TickContext &ctx,
        const rsim_plugin::ActorState &ego,
        const PlanningStartResult &startResult,
        const CartesianFrenetState &frenet,
        bool slSuccess);
    void OpenEgoTrajectoryCsv();
    void WriteEgoTrajectoryCsv(
        const rsim_plugin::TickContext &ctx,
        const rsim_plugin::ActorState &ego,
        std::size_t targetIndex,
        const std::vector<DpPathPoint> &localFrenetPath,
        const std::vector<PlanningTrajectoryPoint> &path);

    std::vector<int32_t> controlled_ids_;
    MapHelper map_;
    std::vector<GlobalPathRouteSegment> route_segments_;
    std::vector<WorldPoint> global_path_world_points_;

    ReferenceLineGenerator reference_line_generator_;
    std::unique_ptr<ReferenceLine> reference_line_;
    bool reference_line_ready_reported_ = false;
    bool reference_line_failure_reported_ = false;

    std::string entity_name_ = "ego";
    std::string route_xosc_path_;
    std::string route_csv_path_;
    std::string reference_line_csv_path_;
    std::string obstacle_csv_path_;
    std::string planning_start_sl_csv_path_;
    std::string ego_trajectory_csv_path_;
    std::FILE *reference_line_csv_fp_ = nullptr;
    std::FILE *planning_start_sl_csv_fp_ = nullptr;
    std::FILE *ego_trajectory_csv_fp_ = nullptr;
    GlobalPathGenerator global_path_generator_;
    ObstacleCsvWriter obstacle_csv_writer_;

    EmPlanner em_planner_;
    std::vector<PlanningTrajectoryPoint> previous_trajectory_;

    double set_speed_ = 8.0;
    bool latched_ = false;
    bool map_loaded_ = false;
    bool initialization_ok_ = false;
};

} // namespace rsim_driver::plugin_internal
