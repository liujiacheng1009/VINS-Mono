#pragma once

#include <string>

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
};
