#pragma once

#include <string>
#include <ros/ros.h>

namespace std_msgs
{
struct Header
{
    ros::Time stamp;
    std::string frame_id;
};
} // namespace std_msgs
