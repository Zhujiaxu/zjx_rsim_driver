#include "GlobalPathGenerator.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <filesystem>
#include <unistd.h>

namespace
{

bool Contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

std::filesystem::path FindRepoRoot()
{
    std::filesystem::path current = std::filesystem::current_path();
    while (true)
    {
        if (std::filesystem::exists(current / "examples" / "DEMO1_rsim_driver_emplanner" / "resource"))
            return current;
        const std::filesystem::path parent = current.parent_path();
        if (parent == current)
            break;
        current = parent;
    }
    return {};
}

}  // namespace

int main()
{
    const std::filesystem::path repoRoot = FindRepoRoot();
    if (repoRoot.empty())
    {
        std::cerr << "cannot locate repo root\n";
        return 1;
    }

    const std::filesystem::path demoRoot =
        repoRoot / "examples" / "DEMO1_rsim_driver_emplanner";
    const std::filesystem::path xodrPath =
        demoRoot / "resource" / "xodr" / "map.xodr";
    const std::filesystem::path xoscPath =
        demoRoot / "resource" / "xosc" / "scene.xosc";

    rsim_driver::MapHelper map;
    if (!map.Load(xodrPath.string()))
    {
        std::cerr << "failed to load map: " << xodrPath << "\n";
        return 1;
    }

    rsim_driver::GlobalPathGenerator generator;
    rsim_driver::GlobalPathResult result;
    if (!generator.GenerateFromXosc(xoscPath.string(), "ego", map, &result))
    {
        std::cerr << "failed to generate global path\n";
        return 1;
    }
    if (result.route_segments.empty() || result.world_points.size() <= 4 ||
        result.chord_length <= 0.0)
    {
        std::cerr << "bad global path result: segments="
                  << result.route_segments.size()
                  << " points=" << result.world_points.size()
                  << " chord=" << result.chord_length << "\n";
        return 1;
    }

    char pathTemplate[] = "/tmp/global_path_smoke_XXXXXX";
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

    if (!rsim_driver::WriteGlobalPathCsv(pathTemplate, result.world_points))
    {
        std::remove(pathTemplate);
        std::cerr << "failed to write csv\n";
        return 1;
    }

    std::ifstream input(pathTemplate);
    std::string header;
    std::string firstRow;
    std::getline(input, header);
    std::getline(input, firstRow);
    std::remove(pathTemplate);

    if (!Contains(header, "index,arc_s,delta_s,x,y") || firstRow.empty())
    {
        std::cerr << "bad csv output\n";
        return 1;
    }

    return 0;
}
