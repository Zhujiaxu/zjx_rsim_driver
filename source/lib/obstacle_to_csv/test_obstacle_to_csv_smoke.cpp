#include "ObstacleToCsv.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

namespace
{

rsim_plugin::ActorState MakeActor(int32_t id,
                                  const char* name,
                                  double x,
                                  double y)
{
    rsim_plugin::ActorState actor{};
    actor.id = id;
    actor.type = 0;
    actor.x = x;
    actor.y = y;
    actor.z = 0.0;
    actor.h = 0.1;
    actor.p = 0.0;
    actor.r = 0.0;
    actor.speed = 2.0;
    actor.vel_x = 1.0;
    actor.vel_y = 0.0;
    actor.vel_z = 0.0;
    actor.acc_x = 0.0;
    actor.acc_y = 0.0;
    actor.acc_z = 0.0;
    actor.length = 4.0;
    actor.width = 2.0;
    actor.height = 1.5;
    actor.road_id = 42;
    actor.lane_id = -1;
    actor.s = 12.0;
    actor.t = -1.0;
    std::strncpy(actor.name, name, sizeof(actor.name) - 1);
    return actor;
}

bool Contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

}  // namespace

int main()
{
    char pathTemplate[] = "/tmp/obstacle_to_csv_smoke_XXXXXX";
    const int fd = mkstemp(pathTemplate);
    if (fd < 0)
    {
        std::cerr << "mkstemp failed\n";
        return 1;
    }
    std::FILE* tempFile = fdopen(fd, "w");
    if (tempFile == nullptr)
    {
        close(fd);
        std::remove(pathTemplate);
        std::cerr << "fdopen failed\n";
        return 1;
    }
    std::fclose(tempFile);

    const rsim_plugin::ActorState ego = MakeActor(2, "ego", 0.0, 0.0);
    const rsim_plugin::ActorState bicycle = MakeActor(7, "bicycle1", 3.0, 4.0);
    const std::vector<rsim_plugin::ActorState> actors = {ego, bicycle};
    const std::vector<int32_t> controlledIds = {ego.id};

    {
        rsim_driver::ObstacleCsvWriter writer;
        if (!writer.Open(pathTemplate))
        {
            std::cerr << "Open failed\n";
            return 1;
        }
        writer.WriteFrame(10, 1.25, ego, actors, controlledIds);
    }

    std::ifstream input(pathTemplate);
    std::string header;
    std::string row;
    std::string extra;
    std::getline(input, header);
    std::getline(input, row);
    std::getline(input, extra);
    std::remove(pathTemplate);

    if (!Contains(header, "frame_id,sim_time,actor_id,actor_name") ||
        !Contains(header, "distance_to_ego"))
    {
        std::cerr << "bad header: " << header << "\n";
        return 1;
    }
    if (!Contains(row, "10,1.250000000,7,bicycle1") ||
        !Contains(row, ",5.000000000"))
    {
        std::cerr << "bad row: " << row << "\n";
        return 1;
    }
    if (Contains(row, "ego") || !extra.empty())
    {
        std::cerr << "controlled actor was not filtered or extra rows exist\n";
        return 1;
    }

    return 0;
}
