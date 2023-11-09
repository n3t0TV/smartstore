#include <ros/ros.h>

#include "mss_ros_utils.h"


std::string
mss_ros_utils::GetNodeName(void)
{
    return ros::this_node::getName();
}


std::string
mss_ros_utils::GetPackageName(void)
{
    std::string package_name;
    const char* kRosPackageNameParamKey = "~package_name";

    if(!ros::param::get(kRosPackageNameParamKey, package_name))
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The \"" << kRosPackageNameParamKey << "\" key"
                       << " couldn't be found in the ROS parameter"
                       << " server.";

        throw std::invalid_argument(err_msg_stream.str());
    }

    return package_name;
}


std::string
mss_ros_utils::GetNodeLogName(void)
{
    static std::string node_log_name;

    if(node_log_name.empty())
    {
        const std::string kSlashChar = "/";
        std::size_t slash_pos;

        node_log_name = GetNodeName();
        slash_pos = node_log_name.rfind(kSlashChar);

        if(slash_pos != std::string::npos)
        {
            slash_pos++; // Skip the slash
            node_log_name = node_log_name.substr(slash_pos);
        }
    }

    return node_log_name;
}


std::string
mss_ros_utils::GetFullLoggerName(void)
{
    return (ROSCONSOLE_ROOT_LOGGER_NAME "." + GetPackageName()
            + "." + GetNodeLogName());
}
