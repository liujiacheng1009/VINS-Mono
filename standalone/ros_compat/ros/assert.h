#pragma once

#include <cassert>
#include <cstdio>

#ifndef ROS_ASSERT
#define ROS_ASSERT(x) assert(x)
#endif

#ifndef ROS_ASSERT_MSG
#define ROS_ASSERT_MSG(cond, ...)               \
    do                                          \
    {                                           \
        if (!(cond))                            \
        {                                       \
            std::fprintf(stderr, __VA_ARGS__);  \
            std::fprintf(stderr, "\n");         \
            assert(cond);                       \
        }                                       \
    } while (0)
#endif

#ifndef ROS_BREAK
#define ROS_BREAK() assert(false)
#endif
