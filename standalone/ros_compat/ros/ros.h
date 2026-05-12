#pragma once

#include <cassert>
#include <cstdio>
#include <sstream>
#include <string>

namespace ros
{
struct Time
{
    double sec;
    Time() : sec(0.0) {}
    explicit Time(double t) : sec(t) {}
    double toSec() const { return sec; }
};

class NodeHandle
{
  public:
    template <typename T>
    bool getParam(const std::string &, T &) const
    {
        return false;
    }

    void shutdown() {}
};
} // namespace ros

#define ROS_COMPAT_STREAM_PRINT(prefix, expr)         \
    do                                                \
    {                                                 \
        std::ostringstream _ros_compat_oss;          \
        _ros_compat_oss << expr;                     \
        std::printf("%s%s\n", prefix, _ros_compat_oss.str().c_str()); \
    } while (0)

#define ROS_INFO(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define ROS_WARN(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define ROS_ERROR(...) do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_DEBUG(...) do {} while (0)
#define ROS_INFO_STREAM(x) ROS_COMPAT_STREAM_PRINT("", x)
#define ROS_WARN_STREAM(x) ROS_COMPAT_STREAM_PRINT("", x)
#define ROS_DEBUG_STREAM(x) do {} while (0)
#define ROS_ERROR_STREAM(x) ROS_COMPAT_STREAM_PRINT("ERROR: ", x)
#define ROS_ASSERT(x) assert(x)
#define ROS_BREAK() assert(false)
