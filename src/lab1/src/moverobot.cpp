#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv , "chatter") ;
	ros::NodeHandle  nh ;
	ros::Publisher pubVel = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel",1000);
	ros::Rate rate(1); //1 command per second
	
	bool isTurning = false;

	while (ros::ok())
	{
	  isTurning = !isTurning;

	  geometry_msgs::Twist msg;
	  if (isTurning) { msg.angular.z = .5; }
	  else { msg.linear.x = 1; }
	  
	  pubVel.publish(msg);	
	  rate.sleep();
	}
}
