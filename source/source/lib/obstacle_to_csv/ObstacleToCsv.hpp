#pragma once

#include "rsim/worldsim_plugin/PluginInterface.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace rsim_driver
{

class ObstacleCsvWriter
{
public:
    ObstacleCsvWriter() = default;
    ~ObstacleCsvWriter();

    ObstacleCsvWriter(const ObstacleCsvWriter&) = delete;
    ObstacleCsvWriter& operator=(const ObstacleCsvWriter&) = delete;

    bool Open(const std::string& path);
    void Close();
    bool IsOpen() const;

    void WriteFrame(uint64_t frameId,
                    double simTime,
                    const rsim_plugin::ActorState& ego,
                    const std::vector<rsim_plugin::ActorState>& actors,
                    const std::vector<int32_t>& controlledIds);

private:
    bool IsControlledActor(int32_t actorId,
                           const std::vector<int32_t>& controlledIds) const;
    void WriteCsvString(const char* value);

    std::FILE* fp_ = nullptr;
    std::string path_;
};

}  // namespace rsim_driver
