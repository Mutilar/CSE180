/*
 *  ROBOTICS, CSE 180
 */

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h> 
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

/* Map Metadata Constants*/
float meters_per_pixel;
int width, height;

/* Boolean representation of Occupancy Grid */
bool** map_matrix;

/* Bounds for relevant map data [ignoring excessive padding on edges of map]*/
int map_x_min, map_x_max, map_y_min, map_y_max;

void getMap(const nav_msgs::OccupancyGrid &map)
{
	meters_per_pixel = map.info.resolution;
	width = map.info.width;
	height = map.info.height;

	bool 



	map_matrix = new bool*[height];
	for (int i = 0; i < height; i++) {
		map_matrix[i] = new bool[width];
	} 

	for (int i = 0; i < width * height; i++) {
		std::cout << "READING POINT(" << i % width << ", " << i / width << ") = " << map.data[i]*1.0 << ";\n";		
		map_matrix[i % width][i / width] = map.data[i] == 0;
	} 
	bool is_all_inaccessible = true;
	for (map_y_min = 0; map_y_min < height && is_all_inaccessible == true; map_y_min++) {
		for (int j = 0; j < width; j++) {
			if (map.data[i, j] != -1) {
				is_all_inaccessible = false;
				break;
			}
		}
	} 
	cout << "test" << map_y_min << "\n";
}



	std::cout << meters_per_pixel << "\n";

	std::cout << "TEST\n";
}

int main(int argc, char **argv)
{
	/* Init Garbage */
	ros::init(argc, argv, "preprocess");
	ros::NodeHandle nh;

	/* Subscribe to Map & Metadata */
	ros::Subscriber subscribe_map = nh.subscribe("/map", 1000, &getMap);

	ros::spin();

	//...

}
