#include <ros/ros.h>

#include "classes/mqttManager.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mqtt_manager_node");
	ros::NodeHandle nh("~");
	MQTTManager mqtt_manager(nh);
	ros::Rate rate(10);
	
	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}
	ros::spinOnce();
	return 0;
}