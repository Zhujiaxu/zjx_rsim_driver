#include "ObstacleToCsv.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rsim_driver
{

ObstacleCsvWriter::~ObstacleCsvWriter()
{
    Close();
}

bool ObstacleCsvWriter::Open(const std::string& path)
{
    Close();
    path_ = path;
    if (path_.empty())
        return true;

    fp_ = std::fopen(path_.c_str(), "w");
    if (fp_ == nullptr)
    {
        std::fprintf(stderr,
                     "[ObstacleCsvWriter] WARNING: cannot open obstacle CSV: %s\n",
                     path_.c_str());
        path_.clear();
        return false;
    }

    std::fprintf(fp_,
                 "frame_id,sim_time,actor_id,actor_name,actor_type,"
                 "x,y,z,h,p,r,speed,"
                 "vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,"
                 "length,width,height,road_id,lane_id,s,t,distance_to_ego\n");
    std::fflush(fp_);
    return true;
}

void ObstacleCsvWriter::Close()
{
    if (fp_ != nullptr)
    {
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

bool ObstacleCsvWriter::IsOpen() const
{
    return fp_ != nullptr;
}

void ObstacleCsvWriter::WriteFrame(
    uint64_t frameId,
    double simTime,
    const rsim_plugin::ActorState& ego,
    const std::vector<rsim_plugin::ActorState>& actors,
    const std::vector<int32_t>& controlledIds)
{
    if (fp_ == nullptr)
        return;

    for (const auto& actor : actors)
    {
        if (IsControlledActor(actor.id, controlledIds))
            continue;

        const double dx = actor.x - ego.x;
        const double dy = actor.y - ego.y;
        const double distanceToEgo = std::sqrt(dx * dx + dy * dy);

        std::fprintf(fp_, "%llu,%.9f,%d,",
                     static_cast<unsigned long long>(frameId),
                     simTime,
                     actor.id);
        WriteCsvString(actor.name);
        std::fprintf(fp_,
                     ",%d,"
                     "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                     "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                     "%.9f,%.9f,%.9f,%d,%d,%.9f,%.9f,%.9f\n",
                     actor.type,
                     actor.x,
                     actor.y,
                     actor.z,
                     actor.h,
                     actor.p,
                     actor.r,
                     actor.speed,
                     actor.vel_x,
                     actor.vel_y,
                     actor.vel_z,
                     actor.acc_x,
                     actor.acc_y,
                     actor.acc_z,
                     actor.length,
                     actor.width,
                     actor.height,
                     actor.road_id,
                     actor.lane_id,
                     actor.s,
                     actor.t,
                     distanceToEgo);
    }

    std::fflush(fp_);
}

bool ObstacleCsvWriter::IsControlledActor(
    int32_t actorId,
    const std::vector<int32_t>& controlledIds) const
{
    return std::find(controlledIds.begin(), controlledIds.end(), actorId) !=
           controlledIds.end();
}

void ObstacleCsvWriter::WriteCsvString(const char* value)
{
    if (value == nullptr)
    {
        std::fprintf(fp_, "\"\"");
        return;
    }

    std::size_t length = 0;
    while (length < 64 && value[length] != '\0')
        ++length;
    bool needsQuotes = false;
    for (std::size_t i = 0; i < length; ++i)
    {
        const char c = value[i];
        if (c == '"' || c == ',' || c == '\n' || c == '\r')
        {
            needsQuotes = true;
            break;
        }
    }

    if (!needsQuotes)
    {
        std::fwrite(value, 1, length, fp_);
        return;
    }

    std::fputc('"', fp_);
    for (std::size_t i = 0; i < length; ++i)
    {
        if (value[i] == '"')
            std::fputc('"', fp_);
        std::fputc(value[i], fp_);
    }
    std::fputc('"', fp_);
}

}  // namespace rsim_driver
