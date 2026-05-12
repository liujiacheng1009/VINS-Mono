#pragma once

#include <cassert>
#include <cstdio>
#include <sstream>

#define VINS_STREAM_PRINT(prefix, expr)                              \
    do                                                               \
    {                                                                \
        std::ostringstream _vins_oss;                                \
        _vins_oss << expr;                                           \
        std::printf("%s%s\n", prefix, _vins_oss.str().c_str());      \
    } while (0)

#define ROS_INFO(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define ROS_WARN(...) do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define ROS_ERROR(...) do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_DEBUG(...) do {} while (0)
#define ROS_INFO_STREAM(x) VINS_STREAM_PRINT("", x)
#define ROS_WARN_STREAM(x) VINS_STREAM_PRINT("", x)
#define ROS_ERROR_STREAM(x) VINS_STREAM_PRINT("ERROR: ", x)
#define ROS_DEBUG_STREAM(x) do {} while (0)
#define ROS_ASSERT(x) assert(x)
#define ROS_ASSERT_MSG(cond, ...)                 \
    do                                            \
    {                                             \
        if (!(cond))                              \
        {                                         \
            std::fprintf(stderr, __VA_ARGS__);    \
            std::fprintf(stderr, "\n");           \
            assert(cond);                         \
        }                                         \
    } while (0)
#define ROS_BREAK() assert(false)
