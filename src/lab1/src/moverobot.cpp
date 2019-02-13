/*
 *	BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  FEB 12, 2019
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "movey-move");
	ros::NodeHandle nh;
	ros::Publisher pubVel = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
	//1 command per second (more of a "square" with a value of ~.4)
	ros::Rate rate(1);
	//Boolean to alternate between driving forward and turning
	bool isTurning = false;

	while (ros::ok())
	{
		isTurning = !isTurning;

		geometry_msgs::Twist msg;
		//rotate .5 rad/s
		if (isTurning) msg.angular.z = .5;
		//move 1 m/s forward
		else msg.linear.x = 1;
		//send message
		pubVel.publish(msg);
		rate.sleep();
	}
}
