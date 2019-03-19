/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  MARCH 18, 2019
 */

 	//logic:

 	//if this == saved position (hasnt moved in 1 second)
		//if at target position
			//next
		//else
			//republish target_pos
			//counter++
			//if counter == 3
				//next (give up)

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>

const double DISTANCE_THRESHOLD = .2;

//Move to Variables
double position[] = {-5, -5, -5, 5, 5, 5, 5, -5};
int nPositions = 4;

int indexPosition = 0; 

int delay_counter = 0;
int retry_counter = 0;

geometry_msgs::PoseStamped target_pos;

ros::Publisher pubMove;
tf::Quaternion myQuaternion;

double old_x, old_y;
double new_x, new_y;
bool is_first_run = true;

void amclGetPose(const geometry_msgs::PoseWithCovarianceStamped &amcl_position)
{
	new_x = amcl_position.pose.pose.position.x;
	new_y = amcl_position.pose.pose.position.y;
	if (is_first_run) {
		old_x = new_x;
		old_y = new_y;
		is_first_run = false;
	}
	// ROS_INFO_STREAM("AMCL AT POSITION(" << new_x << ", " << new_y << ")");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigatey");
	ros::NodeHandle nh;

	ros::Subscriber subAMCLPoint = nh.subscribe("/amcl_pose", 1000, &amclGetPose);	
	pubMove = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
	
	//Init target_pos message... 
	myQuaternion.setRPY( 0, 0, 0 );
	geometry_msgs::Quaternion quat_geo;
	tf::quaternionTFToMsg(myQuaternion, quat_geo);
	target_pos.pose.orientation = quat_geo;
	target_pos.header.frame_id = "map";
	target_pos.pose.position.x = position[indexPosition * 2];
	target_pos.pose.position.y = position[(indexPosition * 2) + 1];

	while (nh.ok()) {
		
		//Check if position has been updated in last second
		double distance_moved = sqrt(((old_x - new_x) * (old_x - new_x)) + ((old_y - new_y) * (old_y - new_y)));
		if (distance_moved > DISTANCE_THRESHOLD) {
			ROS_INFO_STREAM("MOVING...");
			delay_counter = 0;
			retry_counter = 0;
		}
		else {

			//Not moving, check if close to target
			double distance_to_target = sqrt(((position[indexPosition * 2] - new_x) * (position[indexPosition * 2] - new_x)) + (( position[(indexPosition * 2) + 1] - new_y) * ( position[(indexPosition * 2) + 1] - new_y)));
			ROS_INFO_STREAM("STOPPED " << distance_to_target << " FROM TARGET");

			if (distance_to_target < DISTANCE_THRESHOLD * 2) {
				ROS_INFO_STREAM("AT TARGET POSITION, NEXT POSITION");
				indexPosition++;
				indexPosition %= nPositions;
				target_pos.pose.position.x = position[indexPosition * 2];
				target_pos.pose.position.y = position[(indexPosition * 2) + 1];
				pubMove.publish(target_pos);
			}
			else {
				delay_counter++;
				ROS_INFO_STREAM("HASN'T MOVED... " << delay_counter << "/3");
				if (delay_counter >= 3) {
					retry_counter++;
					ROS_INFO_STREAM("STUCK/GOAL REACHED... RESENDING COMMAND... " << retry_counter << "/3");
					pubMove.publish(target_pos);
					ROS_INFO_STREAM("MOVING TOWARDS("<< position[indexPosition * 2] << "," << position[indexPosition * 2 + 1]<< ")");
					delay_counter = 0;
					if (retry_counter >= 3) {
						ROS_INFO_STREAM("STUCK!");
					}
				}
		
			}
		}
		
		//Update position to current position
		old_x = new_x;
		old_y = new_y;

		//Wait, check for new position, and rerun
		ros::Duration(.5).sleep();	
		ros::spinOnce();
		ros::Duration(.5).sleep();	
	}
}

