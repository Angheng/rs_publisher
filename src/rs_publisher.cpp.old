#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "image_transport/image_transport.h"

#include<iostream>


int main (int argc, char* argv[]) try
{
	ros::init(argc, argv, "rs_publisher");
	ros::NodeHandle node;
	ros::Rate rate(10);
	
	std::string name;
	node.getParam("/rs_publisher/name", name);
	ROS_WARN("REALSENSE START. SENSOR NAME : %s", name.c_str());

	ros::Publisher sender_distance = node.advertise<std_msgs::String> (name + "/distance", 1000);

	image_transport::ImageTransport it(node);
	image_transport::Publisher sender_depth_map = it.advertise(name + "/depth_map", 1000);

	rs2::pipeline p;
    p.start();

    while(ros::ok())
    {
        // get depth frame from realsense
		rs2::frameset frames = p.wait_for_frames();
		rs2::depth_frame depth_map = frames.get_depth_frame();
		float dist = depth_map.get_distance(depth_map.get_width() / 2, depth_map.get_height() / 2);
		
		// make image msg using depth frame
		sensor_msgs::Image img_msg;
		img_msg.header.stamp = ros::Time::now();
		img_msg.header.frame_id = name + "_frame";
		img_msg.height = depth_map.get_height();
		img_msg.width = depth_map.get_width();
		img_msg.encoding = "16UC1";
		img_msg.is_bigendian=false;
		img_msg.step = depth_map.get_width() * 2;
		size_t size = img_msg.step * depth_map.get_height();
		img_msg.data.resize(size);
		memcpy(
			(char*)(&img_msg.data[0]), depth_map.get_data(), size
		);

		// msg for center pixel's distance
		std_msgs::String msg;
		msg.data = std::to_string(dist);

		sender_depth_map.publish(img_msg);
		sender_distance.publish(msg);

		rate.sleep();
    }

    return EXIT_SUCCESS;
}

// ROS Error Logger
catch (const rs2::error &e)
{
    ROS_ERROR(
		"ERROR OCCURRED :: %s (%s):\n	%s\n",
		e.get_failed_function().c_str(),
		e.get_failed_args().c_str(),
		e.what()
	);
	
	return EXIT_FAILURE;
}

// RS Exception logger
catch (const std::exception& e)
{
    ROS_ERROR(
		"EXCEPTION :: %s", e.what()
	);

    return EXIT_FAILURE;
}
