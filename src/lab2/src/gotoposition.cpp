/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  FEB 15, 2019
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

//Move to Variables
float X = 5;
float Y = -6;
float THETA = -3.1415/2;

float ACCURACY_THRESHOLD = .2;

//Store local copies of most recent poses
geometry_msgs::Pose basePos;
//Store copy of command
geometry_msgs::Twist msg;

//Publisher for /posedrift
ros::Publisher pubVel;


//On Receiving Pose Ground Truth reading
void positionReceived(const nav_msgs::Odometry &pos)
{
	//Update local copy of Pose Ground Truth
	basePos = pos.pose.pose;

	//float theta_test = tr.euler_from_quaternion(msg.pose.pose.orientation, "sxyz");
	

	double roll,pitch,yaw;

	//From answers.ros.org [convert the yaw Euler angle into the range...]
	tf::Quaternion q(basePos.orientation.x, basePos.orientation.y, basePos.orientation.z, basePos.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	//ROS_INFO_STREAM(yaw);
	if (ros::ok())
	{
		float x_base = basePos.position.x, y_base = basePos.position.y;
		double distance_to_target = sqrt(((x_base - X) * (x_base - X)) + ((y_base - Y) * (y_base - Y)));
		
		if (distance_to_target < ACCURACY_THRESHOLD) {
			//Rotate towards target X,Y
			double difference_in_angles = yaw - THETA;
			if (difference_in_angles < 0) difference_in_angles = -difference_in_angles;

			if (difference_in_angles > ACCURACY_THRESHOLD) {  
				msg.angular.z = .15;
				msg.linear.x = 0;
				ROS_INFO_STREAM("WENT TO POSITION(" << X << "," << Y << "), Angling!");
			}
			else {
				msg.linear.x = 0;
				msg.angular.z = 0;
				ROS_INFO_STREAM("WENT TO POSITION(" << X << "," << Y << ") with Angle(" << THETA << ")");
			}
		}
		else {
			//Find angle towards target X,Y from current X,Y
			double angle = atan2(Y - y_base, X - x_base);
			
			//Rotate to Theta
			msg.angular.z = .15;
			double difference_in_angles = yaw - angle;
			if (difference_in_angles < 0) difference_in_angles = -difference_in_angles;
				
			//ROS_INFO_STREAM(distance_to_target << ", " << angle << "=?=" << yaw << " goes to " << difference_in_angles );
			//If within threshold to theta, move forward
			if (difference_in_angles < ACCURACY_THRESHOLD) {
				ROS_INFO_STREAM("MOVING TOWARDS POSITION(" << X << "," << Y << ")");
				//move forward
				msg.linear.x = .5;
				msg.angular.z = 0;
			}
			else {
				ROS_INFO_STREAM("ROTATING TOWARDS POSITION(" << X << "," << Y << ")");
				msg.linear.x = 0;
			}
		}
		pubVel.publish(msg);
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goeygo");
	ros::NodeHandle nh;
	//Initialize Subscriptions
	ros::Subscriber subPosition = nh.subscribe("/base_pose_ground_truth", 1000, &positionReceived);
	//Initialize Publisher
	pubVel = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
	//Listen to Subscription
	ros::spin();
}
