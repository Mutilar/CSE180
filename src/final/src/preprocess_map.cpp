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
#include <iostream>
#include <fstream>

using namespace std;

/* Map Metadata Constants*/
float meters_per_pixel;
int width, height;

/* Boolean representation of Occupancy Grid */
bool** map_matrix;

/* Iterator to brute-force scan for end-caps of barriers*/
float** cap_iterator;
int cap_iterator_width, cap_iterator_height;

/* Bounds for relevant map data [ignoring excessive padding on edges of map]*/
int map_x_min, map_x_max, map_y_min, map_y_max;

void getMap(const nav_msgs::OccupancyGrid &map)
{
	/* Reading Metadata */
	meters_per_pixel = map.info.resolution;
	width = map.info.width;
	height = map.info.height;

	/* Defining Relevant Boundaries (given 20m x 20m space) */
	map_x_min = (width / 2) - (10 / meters_per_pixel);
	map_x_max = (width / 2) + (10 / meters_per_pixel);

	map_y_min = (height / 2) - (10 / meters_per_pixel);
	map_y_max = (height / 2) + (10 / meters_per_pixel);

	/* Initializing && Populating Map Matrix */
	map_matrix = new bool*[height];
	for (int i = 0; i < height; i++) map_matrix[i] = new bool[width];
	for (int i = 0; i < width * height; i++) map_matrix[i / width][i % width] = map.data[i] == 0;	

	/* Applying Box Filter to Matrix to smooth inperfections in scan */
	for (int num_passes = 0; num_passes < 1; num_passes++) {
		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				int num_available_space = 0;
				if (map_matrix[x+1][y+0]) num_available_space++;
				if (map_matrix[x+0][y+1]) num_available_space++;
				if (map_matrix[x+1][y+1]) num_available_space++;

				if (map_matrix[x-1][y-0]) num_available_space++;
				if (map_matrix[x-0][y-1]) num_available_space++;
				if (map_matrix[x-1][y-1]) num_available_space++;

				if (map_matrix[x+1][y-1]) num_available_space++;
				if (map_matrix[x-1][y+1]) num_available_space++;
				if (num_available_space >= 5) {
					map_matrix[x+0][y+0] = true;
				}
				if (num_available_space <= 3) {
					map_matrix[x+0][y+0] = false;
				}
			}	
		}
	}

	/* Defining "end cap" detection iterator */
	cap_iterator_height = 5;
	cap_iterator_width = 10;
	cap_iterator = new float*[cap_iterator_width];
	for (int i = 0; i < cap_iterator_width; i++) cap_iterator[i] = new float[cap_iterator_height];

	/*	1 	1 	1 	1 	1 	1 	1 	1 	1 	1 */
	/*	1 	1 	1 	1 	1 	1 	1 	1 	1 	1 */
	/*	1 	? 	? 	0 	0 	0 	0 	? 	? 	1 */
	/*	1 	? 	? 	0 	0 	0 	0 	? 	? 	1 */
	/*	1 	? 	0 	0 	0 	0 	0 	0 	? 	1 */
		
	for (int y = 0; y < cap_iterator_height; y++) {
		for (int x = 0; x < cap_iterator_width; x++) {
			if (y == 0 || y == 1 || x == 0 || x == cap_iterator_width - 1) cap_iterator[x][y] = 1;
			else if ((x == 1 || x == cap_iterator_width - 2) && (y == 2 || y == 3 || y == 4)) cap_iterator[x][y] = .5;	
			else if ((x == 2 || x == cap_iterator_width - 3) && (y == 2 || y == 3)) cap_iterator[x][y] = .5;	
			else cap_iterator[x][y] = 0;
			std::cout << cap_iterator[x][y] << " ";
		}
		std:cout<<"\n";
	}


	//Scan for a match with iterator

		//if match, continue setting 0s in current direction until hits black...

	//rotate iterator by 90, repeat x 3





	ofstream myfile;
  	myfile.open ("plot.txt");
  	if (myfile.is_open())
  	{

		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				myfile << map_matrix[x][y] << " ";
				//std::cout << map_matrix[x][y] << " ";
			}
			myfile << "\n";
			//std::cout << "\n";
		}
	 }
  	else cout << "Unable to open file";
	myfile.close();

	std::cout << meters_per_pixel << "\n";

	std::cout << "TEST\n";



	//make sure to garbage collect matrices here!



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
