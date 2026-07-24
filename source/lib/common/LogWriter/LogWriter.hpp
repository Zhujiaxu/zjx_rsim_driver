#pragma once

#include <cstdarg>
#include <cstdio>

namespace rsim_driver
{

inline std::FILE *&PluginLogFilePtr()
{
    static std::FILE *fp = nullptr;
    return fp;
}

inline void SetPluginLogFile(std::FILE *fp)
{
    PluginLogFilePtr() = fp;
}

inline std::FILE *PluginLogFile()
{
    return PluginLogFilePtr();
}

// 仅写日志文件（不输出终端）
inline void PluginLog(const char *fmt, ...)
{
    std::FILE *fp = PluginLogFilePtr();
    if (fp == nullptr)
        return;

    std::va_list args;
    va_start(args, fmt);
    std::vfprintf(fp, fmt, args);
    va_end(args);
    std::fflush(fp);
}

// 写日志文件 + 同步输出到 stderr（EmPlanner 双写专用）
inline void PluginLogEcho(const char *fmt, ...)
{
    // 先输出到 stderr
    std::va_list args1;
    va_start(args1, fmt);
    std::vfprintf(stderr, fmt, args1);
    va_end(args1);

    // 再写日志文件
    std::FILE *fp = PluginLogFilePtr();
    if (fp != nullptr)
    {
        std::va_list args2;
        va_start(args2, fmt);
        std::vfprintf(fp, fmt, args2);
        va_end(args2);
        std::fflush(fp);
    }
}

} // namespace rsim_driver
