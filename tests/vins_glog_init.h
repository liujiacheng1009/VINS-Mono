#pragma once

#include <log_value/value_logger.h>

namespace vins_test_log
{
inline void ensureInitialized()
{
    static bool initialized = false;
    if (initialized)
        return;
    logging::ValueLogger::Options opts;
    opts.program_name = "vins_multi_tests";
    opts.also_log_to_stderr = true;
    logging::ValueLogger::Init(opts);
    initialized = true;
}

struct Init
{
    Init() { ensureInitialized(); }
};

inline Init g_init;
} // namespace vins_test_log
