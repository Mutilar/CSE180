/*
 *  BRIAN HUNGERMAN
 *  ROBOTICS, CSE 180
 *  FEB 22, 2019
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

	ros::init(argc, argv, "transformy");
	ros::NodeHandle nh;

	//Instructions said to subscribe to tf && tf_static, but this is not necessary, as lookUpTransform does not need to subscribe to have global visibility of frames

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	while (nh.ok()) {
	
		geometry_msgs::TransformStamped transformOut;
		geometry_msgs::TransformStamped transformOut2;
       	
       	try {
       		 //In the same tree, grab transform of front_sonar in back_sonar's frame
        	 transformOut = tfBuffer.lookupTransform("front_sonar", "back_sonar", ros::Time(0));
       	}
       	catch (tf2::TransformException &ex) {
       		ROS_WARN("%s",ex.what());
         	ros::Duration(1.0).sleep();
         	continue;
       	}

       	//Build rotation matrix from resulting quaternion
       	tf::Quaternion q(transformOut.transform.rotation.x, transformOut.transform.rotation.y, transformOut.transform.rotation.z, transformOut.transform.rotation.w);
		tf::Matrix3x3 rotationOut(q);
		
		//Print out transformation matrix
		ROS_INFO_STREAM("\nFRAME('front_sonar') from FRAME('back_sonar'):"<<
			"\n|\t"<<rotationOut.getRow(0).x()<<"\t"<<rotationOut.getRow(0).y()<<"\t"<<rotationOut.getRow(0).z()<<"\t"<<transformOut.transform.translation.x<<"\t|"<<
			"\n|\t"<<rotationOut.getRow(1).x()<<"\t"<<rotationOut.getRow(1).y()<<"\t"<<rotationOut.getRow(1).z()<<"\t"<<transformOut.transform.translation.y<<"\t|"<<
			"\n|\t"<<rotationOut.getRow(2).x()<<"\t"<<rotationOut.getRow(2).y()<<"\t"<<rotationOut.getRow(2).z()<<"\t"<<transformOut.transform.translation.z<<"\t|"<<
			"\n|\t0\t0\t0\t1\t|\n");


		//Since there is a disconnect between pioneer and tf_static's frames, grab the two separate conversions and combine them together
		try {
        	 transformOut = tfBuffer.lookupTransform("pioneer/base_link", "pioneer/odom", ros::Time(0));
        	 transformOut2 = tfBuffer.lookupTransform("front_sonar", "base_link", ros::Time(0));
       	}
       	catch (tf2::TransformException &ex) {
       		ROS_WARN("%s",ex.what());
         	ros::Duration(1.0).sleep();
         	continue;
       	}

       	tf::StampedTransform tf_1;
       	tf::StampedTransform tf_2;
       	
       	//Convert from geometry msg to tf to do transformation multiplication
       	tf::transformStampedMsgToTF(transformOut, tf_1);
       	tf::transformStampedMsgToTF(transformOut2, tf_2);

       	//Get resulting transform
       	tf::Transform tf_result = tf_1 * tf_2;
       	tf_1.setData(tf_result);

       	//Convert back to geometry msg to print
       	tf::transformStampedTFToMsg(tf_1, transformOut);

       	//Get rotation matrix
       	tf::Quaternion q2(transformOut.transform.rotation.x, transformOut.transform.rotation.y, transformOut.transform.rotation.z, transformOut.transform.rotation.w);		
		tf::Matrix3x3 rotationOut2(q2);
		
		//Print out transformation matrix
		ROS_INFO_STREAM("\nFRAME('front_sonar') from FRAME('pioneer/odom'):"<<
			"\n|  "<<rotationOut2.getRow(0).x()<<"\t\t"<<rotationOut2.getRow(0).y()<<"\t\t"<<rotationOut2.getRow(0).z()<<"\t\t"<<transformOut.transform.translation.x<<"\t|"<<
			"\n|  "<<rotationOut2.getRow(1).x()<<"\t\t"<<rotationOut2.getRow(1).y()<<"\t\t"<<rotationOut2.getRow(1).z()<<"\t\t"<<transformOut.transform.translation.y<<"\t|"<<
			"\n|  "<<rotationOut2.getRow(2).x()<<"\t\t"<<rotationOut2.getRow(2).y()<<"\t\t"<<rotationOut2.getRow(2).z()<<"\t\t"<<transformOut.transform.translation.z<<"\t|"<<
			"\n|  0\t\t\t0\t\t\t0\t\t\t1\t\t|\n");
	}
}
