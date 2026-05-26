#pragma once

#include <cstdint>
#include <string>

using FrameId = int64_t;
constexpr FrameId kInvalidFrameId = -1;

struct SimpleTime
{
    double sec = 0.0;
    SimpleTime() = default;
    explicit SimpleTime(double t) : sec(t) {}
    double toSec() const { return sec; }
};

struct SimpleHeader
{
    SimpleTime stamp;
    std::string frame_id;
    FrameId seq = kInvalidFrameId;
};
