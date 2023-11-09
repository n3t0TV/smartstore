#include <ros/ros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <stdint.h>

#include <unistd.h>
#include <wiringPi.h>

#include <std_msgs/Bool.h>

#define LOCK_OPEN_GPIO       27    // GPIO ?, WiringPi ?
#define LOCK_FEEDBACK_GPIO   28    // GPIO 20, WiringPi 28

using namespace std;

class LockDriver
{

    public:
        LockDriver(ros::NodeHandle);
        ~LockDriver();
        void LockFeedbackCallback();
        uint16_t LOCK_OPENING_MAX_MILLISECONDS;
        uint16_t LOCK_OPENING_MAX_ATTEMPTS;

    private:
        ros::NodeHandle nh;
        ros::Publisher lockFeedbackPub;
        ros::Subscriber lockOpenSub;

        string node_name;
        std_msgs::Bool lockFeedback;

        void LockOpenCallback(const std_msgs::Bool& msg);        
};

LockDriver::LockDriver(ros::NodeHandle nh_priv)
{
    node_name = ros::this_node::getName();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    wiringPiSetup();
    pinMode(LOCK_OPEN_GPIO, OUTPUT);
    pinMode(LOCK_FEEDBACK_GPIO, INPUT);

    lockFeedback.data = false;

    lockOpenSub = nh.subscribe("/lock_open_topic", 1, &LockDriver::LockOpenCallback, this);

    lockFeedbackPub = nh.advertise<std_msgs::Bool>("/sensor_topic/lid", 1);    

    LockDriver::LOCK_OPENING_MAX_MILLISECONDS = 250;
    LockDriver::LOCK_OPENING_MAX_ATTEMPTS = 5;
}

void LockDriver::LockOpenCallback(const std_msgs::Bool& msg)
{
    if (msg.data) {
        int attempts = 0;
        
        if(!digitalRead(LOCK_FEEDBACK_GPIO)) {

            while(attempts != LockDriver::LOCK_OPENING_MAX_ATTEMPTS) {
                int ms_counter = 0;
                bool lock_opened = true;

                while(!digitalRead(LOCK_FEEDBACK_GPIO)){
                    digitalWrite(LOCK_OPEN_GPIO,1);
                    usleep(1000);
                    if(ms_counter == LockDriver::LOCK_OPENING_MAX_MILLISECONDS){
                        lock_opened = false;
                        break;
                    }
                    ms_counter++;
                }
                digitalWrite(LOCK_OPEN_GPIO,0);
                if(lock_opened) {
                    ROS_DEBUG_STREAM("Lock opened in approximately " << ms_counter << " [ms] at attempt " << attempts+1 << ".");
                    break;
                } else {
                    attempts++;
                    if(attempts == LockDriver::LOCK_OPENING_MAX_ATTEMPTS){
                        ROS_DEBUG("Lock didn't open. Max number of retries reached");
                    } else {
                        ROS_DEBUG("Lock didn't open, retrying...");        
                    }
                }
            }

        } else {
            ROS_DEBUG("Lock was already opened");
        }

    } else {
        ROS_ERROR("Lock open callback error in data");
    }
}

void LockDriver::LockFeedbackCallback()
{
    if(digitalRead(LOCK_FEEDBACK_GPIO) != lockFeedback.data)
    {
        lockFeedback.data = !lockFeedback.data;
        lockFeedbackPub.publish(lockFeedback);
    }
}

LockDriver::~LockDriver()
{
}
