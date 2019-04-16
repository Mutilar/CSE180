/*
 *  ROBOTICS FINAL, CSE 180, SPRING 2019
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h> 
#include <iostream>
#include <fstream>

using namespace std;

/* Stop Spinning on receiving of map */
bool map_has_been_received = false;
bool new_map_has_been_received = false;

/* Map Metadata Constants */
float meters_per_pixel;
int width, height;

/* Boolean representation of Occupancy Grid */
bool** map_matrix;
bool** new_map_matrix;

/* Iterator to brute-force scan for end-caps of barriers */
float** cap_iterator;
int cap_iterator_width, cap_iterator_height;

/* Bounds for relevant map data [ignoring excessive padding on edges of map] */
int map_x_min, map_x_max, map_y_min, map_y_max;

/* Sets of two, target x, target y for robot to move towards (output of node) */
vector<float> target_points;

/* Smoothing Matrix of erronous/noisy points */
void boxFilter();
void newBoxFilter();

/* Writing Matrix to file for viewing */
void plotMatrix();
void plotNewMatrix();

/* Rotating Iterator Matrix 90 degrees */
float** rotateIterator(float** input, int w, int h);

/* Recursively flooding to fill rooms, calculate midpoint */
void floodArea(int x, int y); 

/* Organizing data collected by floodArea() */
struct FloodData {
	int x_total, y_total;
	int count;
};
FloodData flood_data;

/* On receiving New Occupancy Grid... */
void getNewMap(const nav_msgs::OccupancyGrid &map)
{
	new_map_has_been_received = true;
	cout << "\nRECEIVED NEW MAP...\n\n\n";

	/* Reading Metadata */
	meters_per_pixel = map.info.resolution;
	width = map.info.width;
	height = map.info.height;

	cout << meters_per_pixel << ", " << width << ", " << height << "\n";

	/* Defining Relevant Boundaries (given 20m x 20m space) */
	cout << map_y_max << ", " << map_y_min << "\n";

	/* Initializing && Populating Map Matrix */
	new_map_matrix = new bool*[height];
	for (int i = 0; i < height; i++) new_map_matrix[i] = new bool[width];
	for (int i = 0; i < width * height; i++) new_map_matrix[i / width][i % width] = map.data[i] == 0;	

	newBoxFilter();
	plotNewMatrix();
}

/* On receiving Occupancy Grid... */
void getMap(const nav_msgs::OccupancyGrid &map)
{
	map_has_been_received = true;
	cout << "\nRECEIVED MAP...\n\n\n";

	/* Reading Metadata */
	meters_per_pixel = map.info.resolution;
	width = map.info.width;
	height = map.info.height;

	cout << meters_per_pixel << ", " << width << ", " << height << "\n";

	/* Defining Relevant Boundaries (given 20m x 20m space) */
	map_x_min = (width / 2) - (10 / meters_per_pixel);
	map_x_max = (width / 2) + (10 / meters_per_pixel);

	map_y_min = (height / 2) - (10 / meters_per_pixel);
	map_y_max = (height / 2) + (10 / meters_per_pixel);


	cout << map_y_max << ", " << map_y_min << "\n";


	/* Initializing && Populating Map Matrix */
	map_matrix = new bool*[height];
	for (int i = 0; i < height; i++) map_matrix[i] = new bool[width];
	for (int i = 0; i < width * height; i++) map_matrix[i / width][i % width] = map.data[i] == 0;	

	boxFilter();
	plotMatrix();
}

void plotNewMatrix() {
	/* Writing to file */
	ofstream myfile;
  myfile.open ("newplot.txt");
  if (myfile.is_open())
  {
		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				myfile << new_map_matrix[x][y] << " ";
			}
			myfile << "\n";
		}
	}
  else cout << "UNABLE TO OPEN PLOT.TXT\n";
	myfile.close();
}
void plotMatrix() {
	/* Writing to file */
	ofstream myfile;
  myfile.open ("plot.txt");
  if (myfile.is_open())
  {
		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				myfile << map_matrix[x][y] << " ";
			}
			myfile << "\n";
		}
	}
  else cout << "UNABLE TO OPEN PLOT.TXT\n";
	myfile.close();
}

void boxFilter() {
	/* If 5 or more neighbors of any pixel are a given value, set pixel to value */
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
				if (num_available_space >= 5) map_matrix[x+0][y+0] = true;
				if (num_available_space <= 3) map_matrix[x+0][y+0] = false;
			}	
		}
	}
}

void newBoxFilter() {
	/* If 5 or more neighbors of any pixel are a given value, set pixel to value */
	for (int num_passes = 0; num_passes < 1; num_passes++) {
		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				int num_available_space = 0;
				if (new_map_matrix[x+1][y+0]) num_available_space++;
				if (new_map_matrix[x+0][y+1]) num_available_space++;
				if (new_map_matrix[x+1][y+1]) num_available_space++;
				if (new_map_matrix[x-1][y-0]) num_available_space++;
				if (new_map_matrix[x-0][y-1]) num_available_space++;
				if (new_map_matrix[x-1][y-1]) num_available_space++;
				if (new_map_matrix[x+1][y-1]) num_available_space++;
				if (new_map_matrix[x-1][y+1]) num_available_space++;
				if (num_available_space >= 5) new_map_matrix[x+0][y+0] = true;
				if (num_available_space <= 3) new_map_matrix[x+0][y+0] = false;
			}	
		}
	}
}

void postProcess() {
	cout << "running\n\n";

	for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				if (map_matrix[x][y] == 0) new_map_matrix[x][y] = 1; 
			}
	}

	//boxFilter();
	plotNewMatrix();
}


int main(int argc, char **argv)
{
	/* Init Garbage */
	ros::init(argc, argv, "preprocess");
	ros::NodeHandle nh;

	/* Subscribe to Map & Metadata */
	ros::Subscriber subscribe_map = nh.subscribe("/map", 1000, &getMap);
	ros::Subscriber subscribe_map_new = nh.subscribe("/myMap", 1000, &getNewMap);
	
	/* Requesting Map once */
	while (new_map_has_been_received == false || new_map_has_been_received == false) {
		ros::Duration(.01).sleep();	
		ros::spinOnce();
	}

	cout << "RECEIVED BOTH MAPS, PROCESSING RESULTS OF SCAN\n\n\n";

	//........

	postProcess();






}
