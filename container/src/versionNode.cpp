#include <ros/ros.h>

#include "classes/versionManager.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "version_manager_node");
	ros::NodeHandle nh("~");
	VersionManager versionManager(nh);
	ros::Rate rate(10);
	
	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}
	ros::spinOnce();
	return 0;
}