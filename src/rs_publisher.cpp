#include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

#include <thread>
#include <unistd.h>


std::string get_host_name (void)
{
	char hostname[1024];
	gethostname(hostname, 1024);
	std::string str_host_name(hostname);
	
	return str_host_name;
}

static rs2::device sensor_init()
{
	rs2::context ctx;
	rs2::device_hub hub(ctx);
	ROS_WARN("WAITING FOR GET REALSENSE DEVICE...");
	rs2::device device = hub.wait_for_device();
	ROS_WARN("DEVICE FOUND. S/N : %s", device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	ROS_WARN("USING USB VERSION : %s", device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

	return device;
}

void set_msg(sensor_msgs::Image* msg, std::string* name, rs2::frameset* frames, bool is_depth)
{
	msg->header.stamp = ros::Time::now();
	msg->header.frame_id = name->c_str();
	msg->is_bigendian = false;
	
	if (is_depth)
	{
		rs2::depth_frame frame = frames->get_depth_frame();
		msg->encoding = "16UC1";
		msg->height = frame.get_height();
		msg->width = frame.get_width();
		msg->step = msg->width * 2;
		msg->data.resize(msg->step * msg->height);
        memcpy(
        (char*) (&msg->data[0]), frame.get_data(), msg->step * msg->height
        );
	}
	else
	{
		rs2::video_frame frame = frames->get_color_frame();
		msg->encoding = "rgb8";
		msg->height = frame.get_height();
		msg->width = frame.get_width();
		msg->step = msg->width * 3;
		msg->data.resize(msg->step * msg->height);
	    memcpy(
	    (char*) (&msg->data[0]), frame.get_data(), msg->step * msg->height
		);
	}
}


int main (int argc, char* argv[]) try
{
	ros::init(argc, argv, "rs_publisher");
	ros::NodeHandle node;
	ros::Rate rate(10);
	
	std::string name = get_host_name();
	ROS_WARN("REALSENSE START. SENSOR NAME : [%s] =============================", name.c_str());
	rs2::device device = sensor_init();

	image_transport::ImageTransport it(node);
	image_transport::Publisher sender_depth_map = it.advertise(name + "/depth_map", 1000);
	image_transport::Publisher sender_image = it.advertise(name + "/image", 1000);

	rs2::pipeline p;
    p.start();
	
	int width, height;
	std::string frame_id = name + "_frame";
	
	ROS_WARN("SENSOR NOW READY. STARTING PROGRESS ==========================================");

    while(ros::ok())
    {
		rs2::frameset frames = p.wait_for_frames();
		
		sensor_msgs::Image image_msg;
		std::thread image_thread = std::thread(set_msg, &image_msg, &frame_id, &frames, false);
		
		sensor_msgs::Image depth_msg;
		std::thread depth_thread = std::thread(set_msg, &depth_msg, &frame_id, &frames, true);

		image_thread.join();
		depth_thread.join();

		sender_image.publish(image_msg);
		sender_depth_map.publish(depth_msg);

		rate.sleep();
    }

    return EXIT_SUCCESS;
}

// ROS Error Logger
catch (const rs2::camera_disconnected_error &e)
{
	ROS_ERROR(
		"CAMERA WAS DISCONNECTED. PLZ CHECK YOUR CAMERA CONNECTION"
	);
}

// RS Exception logger
catch (const std::exception& e)
{
    ROS_ERROR(
		"EXCEPTION :: %s", e.what()
	);

    return EXIT_FAILURE;
}
