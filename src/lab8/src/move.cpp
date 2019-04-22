/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  MARCH 22, 2019
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
//Given
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


void setGoal();

//Area of Interest
const double X_MIN = 4, X_MAX = 9, Y_MIN = 0, Y_MAX = 6;
double target_x, target_y;

bool is_shifting = true;
bool is_moving_y_positive = true;

move_base_msgs::MoveBaseGoal goal;


void setGoal() {

	ROS_INFO_STREAM("TARGET(" << target_x << ", " << target_y << ")");
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base");
	
	while (!ac.waitForServer()) { }
	
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.position.x = target_x;
	goal.target_pose.pose.position.y = target_y;

	goal.target_pose.pose.orientation.w = 1.0;

	ac.sendGoal(goal);
	ac.waitForResult();

	is_shifting = !is_shifting;
	if (is_shifting) {
		target_x++;
		if (target_x > X_MAX) {
			ros::shutdown();
		}
	}
	else {
		is_moving_y_positive = !is_moving_y_positive;
		if (is_moving_y_positive) {
			target_y = Y_MAX;
		}
		else {
			target_y = Y_MIN;
		}

	}
	setGoal();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "movemove");
	ros::NodeHandle nh;
	
	target_x = X_MIN;
	target_y = Y_MAX;

	setGoal();

	ros::spin();

	return 0;
}

