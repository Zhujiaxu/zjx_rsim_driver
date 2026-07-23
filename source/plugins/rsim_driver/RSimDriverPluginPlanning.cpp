#include "RSimDriverPlugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace rsim_driver::plugin_internal
{

namespace
{

constexpr double kFallbackDeceleration = 6.0;
constexpr double kMinTimeStep = 1e-6;

} // namespace

bool RSimDriverPlugin::RunEmPlannerDetailed(
    const rsim_plugin::TickContext &ctx,
    const rsim_plugin::ActorState &ego,
    EmPlannerResult *result)
{
    if (result == nullptr || reference_line_ == nullptr)
        return false;

    EmPlannerResult output;
    if (!em_planner_.EMPlanPathDetailed(ctx.actors,
                                        ego.id,
                                        ego,
                                        ctx.sim_time,
                                        previous_trajectory_,
                                        reference_line_->points,
                                        &output))
    {
        *result = std::move(output);
        return false;
    }

    if (!em_planner_.EMPlanSpeedDetailed(ctx.actors, ego.id, &output))
    {
        *result = std::move(output);
        return false;
    }

    if (!em_planner_.EMPlanPostProcessDetailed(output))
    {
        *result = std::move(output);
        return false;
    }

    *result = std::move(output);
    return true;
}

bool RSimDriverPlugin::InstallGlobalPathFromXosc(
    const std::string &xoscPath)
{
    GlobalPathResult result;
    if (!global_path_generator_.GenerateFromXosc(
            xoscPath, entity_name_, map_, &result))
    {
        return false;
    }

    route_segments_ = std::move(result.route_segments);
    global_path_world_points_.clear();
    global_path_world_points_.reserve(result.world_points.size());
    for (const auto &point : result.world_points)
    {
        global_path_world_points_.push_back({point.x, point.y});
    }

    const bool csvWritten =
        global_path_generator_.WriteCsv(route_csv_path_, result.world_points);
    if (csvWritten && !route_csv_path_.empty())
    {
        std::fprintf(stderr,
                     "[RSimDriver] global path CSV written: %s "
                     "(rows=%zu, chord=%.2f m)\n",
                     route_csv_path_.c_str(),
                     global_path_world_points_.size(),
                     result.chord_length);
    }
    return true;
}

const rsim_plugin::ActorState *RSimDriverPlugin::FindControlledActor(
    const rsim_plugin::TickContext &ctx) const
{
    for (const auto &actor : ctx.actors)
    {
        if (actor.id == controlled_ids_.front())
            return &actor;
    }
    return nullptr;
}

rsim_plugin::ActorUpdate RSimDriverPlugin::BuildStopActorUpdate(
    const rsim_plugin::ActorState &ego) const
{
    rsim_plugin::ActorUpdate update;
    update.actor_id = ego.id;
    update.x = ego.x;
    update.y = ego.y;
    update.z = ego.z;
    update.h = ego.h;
    update.p = ego.p;
    update.r = ego.r;
    update.speed = 0.0;
    update.wheel_angle = 0.0;
    update.position_valid = 1;
    update.speed_valid = 1;
    return update;
}

rsim_plugin::ActorUpdate RSimDriverPlugin::BuildControlledStopActorUpdate(
    const rsim_plugin::ActorState &ego,
    double dt) const
{
    rsim_plugin::ActorUpdate update;
    update.actor_id = ego.id;

    const double timeStep = dt > kMinTimeStep ? dt : 0.0;
    const double speed =
        std::isfinite(ego.speed) ? std::max(0.0, ego.speed) : 0.0;
    const double nextSpeed =
        std::max(0.0, speed - kFallbackDeceleration * timeStep);
    const double distance = 0.5 * (speed + nextSpeed) * timeStep;
    const double heading = std::isfinite(ego.h) ? ego.h : 0.0;

    update.x = ego.x + distance * std::cos(heading);
    update.y = ego.y + distance * std::sin(heading);
    update.z = ego.z;
    update.h = heading;
    update.p = ego.p;
    update.r = ego.r;
    update.speed = nextSpeed;
    update.wheel_angle = 0.0;
    update.position_valid = 1;
    update.speed_valid = 1;
    return update;
}

rsim_plugin::ActorUpdate
RSimDriverPlugin::BuildActorUpdateFromTrajectoryPoint(
    const rsim_plugin::ActorState &ego,
    const PlanningTrajectoryPoint &target) const
{
    rsim_plugin::ActorUpdate update;
    update.actor_id = ego.id;
    update.x = target.x;
    update.y = target.y;
    update.z = ego.z;
    update.h = target.heading;
    update.p = ego.p;
    update.r = ego.r;
    update.speed = target.speed;
    update.wheel_angle = 0.0;
    update.position_valid = 1;
    update.speed_valid = 1;
    return update;
}

void RSimDriverPlugin::LatchInitialState(const rsim_plugin::ActorState &ego)
{
    if (latched_)
        return;

    if (!route_segments_.empty())
    {
        const GlobalPathRouteSegment &first = route_segments_.front();
        const int routeLane =
            first.lane_id != 0
                ? first.lane_id
                : map_.TrackTToLane(first.road_id, first.s_start, first.t);

        const double dx = global_path_world_points_.empty()
                              ? 0.0
                              : ego.x - global_path_world_points_.front().x;
        const double dy = global_path_world_points_.empty()
                              ? 0.0
                              : ego.y - global_path_world_points_.front().y;
        const double startError = std::sqrt(dx * dx + dy * dy);
        if (startError > 2.0)
        {
            std::fprintf(stderr,
                         "[RSimDriver] WARNING: initial ego (%.2f, %.2f) is "
                         "%.2f m from the global path start\n",
                         ego.x,
                         ego.y,
                         startError);
        }

        std::fprintf(stderr,
                     "[RSimDriver] route start road=%lld lane=%d s=%.2f "
                     "world=(%.2f, %.2f) ego=(%.2f, %.2f)\n",
                     static_cast<long long>(first.road_id),
                     routeLane,
                     first.s_start,
                     global_path_world_points_.empty()
                         ? 0.0
                         : global_path_world_points_.front().x,
                     global_path_world_points_.empty()
                         ? 0.0
                         : global_path_world_points_.front().y,
                     ego.x,
                     ego.y);
    }

    latched_ = true;
}

void RSimDriverPlugin::UpdateReferenceLine(
    const rsim_plugin::ActorState &ego)
{
    if (global_path_world_points_.empty())
        return;

    std::unique_ptr<ReferenceLine> generated =
        reference_line_generator_.Generate(
            global_path_world_points_, ego.x, ego.y);
    if (!generated)
    {
        reference_line_.reset();
        if (!reference_line_failure_reported_)
        {
            std::fprintf(stderr,
                         "[RSimDriver] WARNING: reference line generation "
                         "failed (worldPoints=%zu)\n",
                         global_path_world_points_.size());
            reference_line_failure_reported_ = true;
        }
        return;
    }

    reference_line_ = std::move(generated);
    reference_line_failure_reported_ = false;
    if (!reference_line_ready_reported_)
    {
        std::fprintf(stderr,
                     "[RSimDriver] reference line ready: points=%zu\n",
                     reference_line_->points.size());
        reference_line_ready_reported_ = true;
    }
}

} // namespace rsim_driver::plugin_internal
