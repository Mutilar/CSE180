/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  MARCH 22, 2019
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

geometry_msgs::PoseStamped target_pos;
double target_rotation;

bool is_first_run = true;

void amclGetPose(const geometry_msgs::PoseWithCovarianceStamped &amcl_position)
{
	target_pos = amcl_position;

	double roll, pitch, yaw;

	tf::Quaternion q(target_pos.pose.orientation.x, target_pos.pose.orientation.y, target_pos.pose.orientation.z, target_pos.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	target_rotation = yaw;

	ROS_INFO_STREAM("AMCL POSITION ( Theta = " << target_rotation << " , X = " << target_pos.pose.position.x << " , Y = " << target_pos.pose.position.y << " )\n");
	is_first_run = false;
}

void getCloud(const geometry_msgs::PoseArray &points)
{
	double total_theta = 0, total_x = 0, total_y = 0, count = 0;

	double roll, pitch, yaw;

	/* Summing Point Cloud points*/
	for (int i = 0; i < points.length; i++)
	{
		tf::Quaternion q(basePos.orientation.x, basePos.orientation.y, basePos.orientation.z, basePos.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		total_theta += yaw;
		total_x += points.poses[i].position.x;
		total_y += points.poses[i].position.y;
		count++;
	}
	ROS_INFO_STREAM("AVERAGED POINT CLOUD ( Theta = " << total_theta / count << " , X = " << total_x / count << " , Y = " << total_y / count << " )\n");

	/* Calculate Difference in Point Cloud && AMCL Pose */
	if (!is_first_run)
	{
		ROS_INFO_STREAM("DIFFERENCE ( Theta = " << target_rotation - total_theta / count << " , X = " << target_pos.pose.position.x - total_x / count << " , Y = " << target_pos.pose.position.y - total_y / count << " )\n");
	}
	else
	{
		ROS_INFO_STREAM("WAITING TO RECEIVE AMCL POSITION\n");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "averagey");
	ros::NodeHandle nh;

	ros::Subscriber subAMCLPoint = nh.subscribe("/particlecloud", 1000, &getCloud);
	ros::Subscriber subAMCLPoint = nh.subscribe("/amcl_pose", 1000, &amclGetPose);

	ros::spin();
	return 0;
}
