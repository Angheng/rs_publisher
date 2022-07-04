#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

#include <unistd.h>

	
std::string
get_host_name (void)
{
     char hostname[1024];
     gethostname(hostname, 1024);
     std::string str_host_name(hostname);
     return str_host_name;
}

void setCamInfo(sensor_msgs::CameraInfo* tmp)
{
	tmp->height = 480;
	tmp->width = 640;
	tmp->distortion_model = "plumb_bob";
	tmp->D = {0.0, 0.0, 0.0, 0.0, 0.0};
	tmp->K = {384.08355712890625, 0.0, 321.8194885253906, 0.0, 384.08355712890625, 238.3449249267578, 0.0, 0.0, 1.0};
	tmp->R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	tmp->P = {384.08355712890625, 0.0, 321.8194885253906, 0.0, 0.0, 384.08355712890625, 238.3449249267578, 0.0, 0.0, 0.0, 1.0, 0.0};
	tmp->binning_x = 0;
	tmp->binning_y = 0;
}


int main (int argc, char* argv[])
{
	ros::init(argc, argv, "cam_info_publisher");
	ros::NodeHandle node;
	ros::Rate rate(10);
	
	std::string name = get_host_name();
	ROS_WARN("Running %s's Camera info publisher", name.c_str());

	ros::Publisher sender_cam_info = node.advertise<sensor_msgs::CameraInfo> (name+ "/camera_info", 1000);
	
	sensor_msgs::CameraInfo cam_info;
	cam_info.header.frame_id = name + "_frame";
	setCamInfo(&cam_info);

    while(ros::ok())
    {
		cam_info.header.stamp = ros::Time::now();
		sender_cam_info.publish(cam_info);

		rate.sleep();
    }

    return 0;
}	
