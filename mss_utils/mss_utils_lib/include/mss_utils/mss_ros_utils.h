#pragma once

#include <string>

#define MSS_ROS_DEBUG(...) \
    ROS_DEBUG_NAMED(mss_ros_utils::GetNodeLogName(), __VA_ARGS__)

#define MSS_ROS_INFO(...) \
    ROS_INFO_NAMED(mss_ros_utils::GetNodeLogName(), __VA_ARGS__)

#define MSS_ROS_WARN(...) \
    ROS_WARN_NAMED(mss_ros_utils::GetNodeLogName(), __VA_ARGS__)

#define MSS_ROS_ERROR(...) \
    ROS_ERROR_NAMED(mss_ros_utils::GetNodeLogName(), __VA_ARGS__)

#define MSS_ROS_FATAL(...) \
    ROS_FATAL_NAMED(mss_ros_utils::GetNodeLogName(), __VA_ARGS__)

namespace mss_ros_utils {
    std::string
    GetNodeName(void);

    std::string
    GetPackageName(void);

    std::string
    GetNodeLogName(void);

    std::string
    GetFullLoggerName(void);
}
