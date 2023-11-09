#include <algorithm>
#include <ros/console.h>

#include "mss_ros_utils.h"
#include "ros_node_wrapper.h"


RosNodeWrapper * RosNodeWrapper::instance_;


RosNodeWrapper::RosNodeWrapper()
{
    instance_ = this;
}


void RosNodeWrapper::Loop(int argc, char **argv, std::string node_name,
                          uint16_t sec_timeout)
{
    uint16_t timeout_cntr = 0U;

    sigint_exit_.store(false);
    signal(SIGINT, SigintHandler);
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);

    ros::Time::init();
    while (!ros::master::check()
           && ((timeout_cntr < sec_timeout) || (sec_timeout == 0U))
           && !sigint_exit_.load())
    {
        timeout_cntr++;
        ROS_DEBUG("Waiting %d second(s) for the ROS master", timeout_cntr);
        ros::Duration(1).sleep();
    }

    if(ros::master::check())
    {
        rn_handle_ = std::make_unique<ros::NodeHandle>();

        if(!ros::isShuttingDown())
        {
            SetLoggerLevel();
            BeforeSpin();
            ros::spin();
            AfterSpin();
        }
    }
    else
    {
        /* If ros::shutdown has already been called, use printf */
        if (ros::ok())
        {
            ROS_ERROR("Couldn't contact with ROS master after %d second(s),"
                      " exiting", timeout_cntr);
        }
        else
        {
            printf("[%s] Couldn't contact with ROS master after %d second(s),"
                   " exiting\n", node_name.c_str(), timeout_cntr);
        }
    }

    /* In case ros::spin is not called, there's no need to call ros::shutdown
     * here, as the node will automatically shutdown when the NodeHandle
     * destructs. */
}


void RosNodeWrapper::SigintHandler(int sig)
{
    instance_->sigint_exit_.store(true);
    instance_->ProcessSigint();
    // The default handler calls shutdown to stop ros::spin
    ros::shutdown();
}


ros::NodeHandle *RosNodeWrapper::GetNodeHandle(void)
{
    return rn_handle_.get();
}


std::string
RosNodeWrapper::GetNodeName(void)
{
    return ros::this_node::getName();
}


void
RosNodeWrapper::SetLoggerLevel(void)
{
    const char* kRosLogLvlNameParamKey = "~log_level";
    int log_level;
    ros::console::levels::Level console_level;

    ros::param::param<int>(kRosLogLvlNameParamKey, log_level,
                           (int) ros::console::levels::Debug);
    log_level = std::min(log_level, (int) ros::console::levels::Fatal);
    log_level = std::max(log_level, (int) ros::console::levels::Debug);
    console_level = (ros::console::levels::Level) log_level;

    if(ros::console::set_logger_level(mss_ros_utils::GetFullLoggerName(),
                                      console_level))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
}
