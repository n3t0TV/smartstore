#include <ros/ros.h>

#include "classes/lockDriver.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"lock_driver_node");
	ros::NodeHandle nh("~");
	LockDriver lock(nh);
	
	ros::Rate rate(5);
	ros::Timer lockFeedbackTimer = nh.createTimer(ros::Duration(1.0),bind(&LockDriver::LockFeedbackCallback,lock));

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	ros::spinOnce();

	return 0;
}
