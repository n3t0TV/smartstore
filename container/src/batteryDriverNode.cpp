#include <ros/ros.h>

#include "classes/batteryDriver.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "barrety_driver_node");
  ros::NodeHandle nh("~");
  BatteryDriver battery(nh);

  ros::Rate rate(1);
  ros::Timer batteryLevelTimer = nh.createTimer(
      ros::Duration(2.0), bind(&BatteryDriver::BatteryLevelCallback, battery));

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}