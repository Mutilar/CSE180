/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  FEB 22, 2019
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>



//void referenceReceived(const tf2_msgs::TFMessage &references)
//{

//	ROS_INFO_STREAM(references);

	/* From wiki.ros.org tutorial on tf listener*/
	
	
	//print out results here?

//}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "transformy");
	ros::NodeHandle nh;

	//Initialize Subscriptions
	//ros::Subscriber subReferences = nh.subscribe("/tf_static", 1000, &referenceReceived);

	//Initialize Publisher
		
	//Listen to Subscription
	//ros::spin();

	tf2_msgs::TFMessage static_references;
	tf::TransformListener listener;


	bool notPrinted = true;
	while (nh.ok() && notPrinted) {
	
		tf::StampedTransform transformOut;
		try {
			listener.lookupTransform("front_sonar", "back_sonar" , ros::Time(0), transformOut);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		ROS_INFO_STREAM(transformOut);
		notPrinted = false;
	}
}
