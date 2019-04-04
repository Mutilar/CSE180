/*
 *  ROBOTICS, CSE 180
 */

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "scan");
	ros::NodeHandle nh;

 	// ros::spin();

	return 0;
}
