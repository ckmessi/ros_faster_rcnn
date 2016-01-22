#include "ros/ros.h"
#include "ros_faster_rcnn/KcfTracker.h"
#include <cstdlib>

#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kcf_tracker_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_faster_rcnn::KcfTracker>("kcf_tracker");
  ros_faster_rcnn::KcfTracker srv;
  
	string command = "";
	string path = "";
	string rectangle = "";
	while(true)
	{	
		cin >> command >> path >> rectangle;
		if(command == "exit"){
			break;
		}

		srv.request.command = command;
		srv.request.path = path;
		srv.request.rectangle = rectangle;
		if (client.call(srv))
		{
			cout << srv.response.debug_info << ":" << srv.response.detection_info << endl;
		}
		else
		{
			ROS_ERROR("Failed to call kcf_tracker");
			return 1;
		}
	}
  return 0;
}

