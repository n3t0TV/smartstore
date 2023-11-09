#include <ros/ros.h>

#include "classes/gpsDriver.h"

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
	init(argc,argv,"gps_driver_node");
	NodeHandle nh("~");
	GpsDriver gps(nh);
	
	Rate rate(5);
	Timer timer = nh.createTimer(Duration(5.0),&GpsDriver::GpsFeedbackCallback,&gps);

	while(ok())
	{
		spinOnce();
		rate.sleep();
	}
	
	spinOnce();

	return 0;
}
