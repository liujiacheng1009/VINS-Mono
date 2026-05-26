#pragma once

#include "utility/simple_types.h"

#include <eigen3/Eigen/Dense>
#include <map>
#include <utility>
#include <vector>

struct ImageFrameInput
{
    FrameId id = kInvalidFrameId;
    SimpleHeader header;
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> features;
};
