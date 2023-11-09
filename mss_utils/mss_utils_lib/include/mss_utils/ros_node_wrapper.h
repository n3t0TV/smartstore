#pragma once

#include <memory>
#include <ros/ros.h>
#include <signal.h>
#include <string>


class RosNodeWrapper
{
    public:
        RosNodeWrapper();


        void Loop(int argc, char **argv, std::string node_name,
                  uint16_t sec_timeout);


        virtual void BeforeSpin(void)
        {
        }


        virtual void AfterSpin(void)
        {
        }


        virtual void ProcessSigint(void)
        {
        }


        ros::NodeHandle *GetNodeHandle(void);


        std::string GetNodeName(void);


    private:
        static RosNodeWrapper *instance_;

        static void SigintHandler(int sig);

        std::atomic<bool> sigint_exit_;

        std::unique_ptr<ros::NodeHandle> rn_handle_;

        void SetLoggerLevel(void);
};
