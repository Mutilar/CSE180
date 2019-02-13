/*
 *	BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  FEB 12, 2019
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

//Store local copies of most recent poses
geometry_msgs::Pose odomPos;
geometry_msgs::Pose basePos;

//Store local copy of return value
std_msgs::Float64 val;

//Publisher for /posedrift
ros::Publisher pubDist;

//On receiving Odometry reading
void odomReceived(const nav_msgs::Odometry &msg)
{
	//Update local copy of Odometry reading
	odomPos = msg.pose.pose;
	//If ready to publish
	if (ros::ok())
	{
		//Calculate distance betwee real and theoretical XY positions
		double x1 = odomPos.position.x, x2 = basePos.position.x, y1 = odomPos.position.y, y2 = basePos.position.y;
		val.data = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
		//Push calculated distance to /posedrift
		pubDist.publish(val);
	}
}

//On Receiving Pose Ground Truth reading
void baseReceived(const nav_msgs::Odometry &msg)
{
	//Update local copy of Pose Ground Truth
	basePos = msg.pose.pose;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drifty-drift");
	ros::NodeHandle nh;
	//Initialize Subscriptions
	ros::Subscriber subOdom = nh.subscribe("/pioneer/odom", 1000, &odomReceived);
	ros::Subscriber subActual = nh.subscribe("/base_pose_ground_truth", 1000, &baseReceived);
	//Initialize Publisher
	pubDist = nh.advertise<std_msgs::Float64>("/posedrift", 1000);

	ros::spin();
}
