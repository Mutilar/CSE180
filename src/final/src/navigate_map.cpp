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
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;

void setGoal();

/* Start movement on receiving commands */
bool has_been_instructed = false;

move_base_msgs::MoveBaseGoal goal;

/* Sets of two, target x, target y for robot to move towards (output of node) */
vector<float> target_points;

void getPoints(const geometry_msgs::Vector3 &point)
{
	std::cout << "RECEIVING POINT(" << point.x << ", " << point.y << ")\n";

	target_points.push_back(point.y);
	target_points.push_back(point.x);

	has_been_instructed = true;
}

void setGoal()
{

	// ROS_INFO_STREAM("TARGET(" << target_x << ", " << target_y << ")");
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");

	ros::NodeHandle nh;
	ros::Publisher pubBool = 
		nh.advertise<std_msgs::Bool>("/navigate_finished",1000);

	cout << "waiting for server...\n";
	while (!ac.waitForServer())	{ }
	cout << "found server...\n";

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	cout << "TARGET( y = " << target_points.back() << ", ";
	goal.target_pose.pose.position.y = target_points.back();
	target_points.pop_back();
	cout << "x = " << target_points.back() << ")\n";
	goal.target_pose.pose.position.x = target_points.back();
	target_points.pop_back();

	goal.target_pose.pose.orientation.w = 1.0;

	ac.sendGoal(goal);
	ac.waitForResult();

	std::cout << target_points.size() << "\n";

	if (target_points.size() == 0)
	{
		//Run PostProcessor
		std_msgs::Bool targetReached;			//Last Target Reached
		targetReached.data = true;				//Publish TRUE to pubBool Topic so that next traget point is added
		pubBool.publish(targetReached);	

		ros::shutdown();
	}
	else
	{
		std_msgs::Bool targetReached;			//Last Target Reached
		targetReached.data = false;				//Publish TRUE to pubBool Topic so that next traget point is added
		pubBool.publish(targetReached);

		setGoal();
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "navigate");
	ros::NodeHandle nh;

	ros::Subscriber subscribe_points = nh.subscribe("/target_points", 1000, &getPoints);
	int spin_more = 0;

	//Run Preprocessor

	while (spin_more < 10)
	{
		ros::Duration(1).sleep();

		for (int i = 0; i < 10; i++)
			ros::spinOnce();

		if (has_been_instructed)
			spin_more++;
	}
	cout << "received instructions\n\n";

	ros::Duration(1).sleep();

	setGoal();
	ros::spin();

	return 0;
}