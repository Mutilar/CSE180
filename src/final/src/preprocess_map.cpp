/*
 *  ROBOTICS FINAL, CSE 180, SPRING 2019
 */

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h> 
#include <iostream>
#include <fstream>

using namespace std;

/* Stop Spinning on receiving of map */
bool has_been_preprocessed = false;

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

/* Sets of two, target x, target y for robot to move towards (output of node) */
vector<float> target_points;

/* Smoothing Matrix of erronous/noisy points*/
void boxFilter();

/* Writing Matrix to file for viewing */
void plotMatrix();

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


/* On receiving Occupancy Grid... */
void getMap(const nav_msgs::OccupancyGrid &map)
{
	has_been_preprocessed = true;
	cout << "\nRECEIVED MAP, PREPROCESSING...\n\n\n";

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
	boxFilter();

	/* Defining "end cap" detection iterator */
	cap_iterator_height = 5;
	cap_iterator_width = 10;
	cap_iterator = new float*[cap_iterator_width];
	for (int i = 0; i < cap_iterator_width; i++) cap_iterator[i] = new float[cap_iterator_height];

	/*	1 	1 	1 	1 	1 	1 	1 	1 	1 	1 */
	/*	1 	1 	1 	1 	1 	1 	1 	1 	1 	1 */
	/*	1 	? 	? 	? 	0 	0 	? 	? 	? 	1 */
	/*	1 	? 	? 	0 	0 	0 	0 	? 	? 	1 */
	/*	1 	? 	0 	0 	0 	0 	0 	0 	? 	1 */

	for (int y = 0; y < cap_iterator_height; y++) {
		for (int x = 0; x < cap_iterator_width; x++) {
			if (y == 0 || y == 1 || x == 0 || x == cap_iterator_width - 1) cap_iterator[x][y] = 1;
			else if ((x == 1 || x == cap_iterator_width - 2) && (y == 2 || y == 3 || y == 4)) cap_iterator[x][y] = .5;	
			else if ((x == 2 || x == cap_iterator_width - 3) && (y == 2 || y == 3)) cap_iterator[x][y] = .5;	
			else if ((x == 3 || x == cap_iterator_width - 4) && (y == 2)) cap_iterator[x][y] = .5;	
			else cap_iterator[x][y] = 0;
		}
	}

	/* Search for matches against iterator */
	for (int rotations = 0; rotations < 4; rotations++) {
		for (int x = map_x_min; x < map_x_max; x++) {
			for (int y = map_y_min; y < map_y_max; y++) {
				bool is_match = true;
				/* Checking if point satisfies condition of Iterator */
				for (int x_it = 0; x_it < cap_iterator_width && is_match; x_it++) {
					for (int y_it = 0; y_it < cap_iterator_height && is_match; y_it++) {		
							if (abs(map_matrix[x + (x_it - cap_iterator_width / 2)][y + (y_it - cap_iterator_height / 2)] * 1.0f - cap_iterator[x_it][y_it]) > .75f ) is_match = false;
					}
				}
				if (is_match) 
				{
					bool has_hit_another_barrier = false;
					/* Controlling which direction to extend in */
					switch (rotations) {
						case 0:
						for (int extender = 1; extender < 500 && has_hit_another_barrier == false; extender++) {
							for (int width_extender = -3; width_extender <= 3; width_extender++) {
								if (map_matrix[x][y-extender - 1] == false) has_hit_another_barrier = true;
								map_matrix[x+width_extender][y-extender] = false;
							}
						}
						break;
						case 1:
						for (int extender = 1; extender < 500 && has_hit_another_barrier == false; extender++) {
							for (int width_extender = -3; width_extender <= 3; width_extender++) {
								if (map_matrix[x-extender - 1][y] == false) has_hit_another_barrier = true;
								map_matrix[x-extender][y+width_extender] = false;
							}
						}
						break;
						case 2:
						for (int extender = 1; extender < 500 && has_hit_another_barrier == false; extender++) {
							for (int width_extender = -3; width_extender <= 3; width_extender++) {
								if (map_matrix[x][y+extender + 1] == false) has_hit_another_barrier = true;
								map_matrix[x+width_extender][y+extender] = false;
							}
						}
						break;
						case 3:
						for (int extender = 1; extender < 500 && has_hit_another_barrier == false; extender++) {
							for (int width_extender = -3; width_extender <= 3; width_extender++) {
								if (map_matrix[x+extender + 1][y] == false) has_hit_another_barrier = true;
								map_matrix[x+extender][y+width_extender] = false;
							}
						}
						break;
					}
					/* Filling in extra points around Iterator... */
					for (int x_it = 0; x_it < cap_iterator_width && is_match; x_it++) {
						for (int y_it = 0; y_it < cap_iterator_height && is_match; y_it++) {		
							if (cap_iterator[x_it][y_it] < 1) map_matrix[x + (x_it - cap_iterator_width / 2)][y + (y_it - cap_iterator_height / 2)] = false;
						}
					}				
				}
			}	
		}
		/* Rotating Iterator 90 degrees */
		cap_iterator = rotateIterator(cap_iterator, cap_iterator_width, cap_iterator_height);
		int temp = cap_iterator_height;
		cap_iterator_height = cap_iterator_width;
		cap_iterator_width = temp;
	}

	/* Reapply a Box Filter to smooth out any new edges */
	boxFilter();
	
	/* Writing preprocessed map to view in GNUPlot */
	plotMatrix();
	
	/* Map Preprocessing Done! */
	cout << "PREPROCESSING FINISHED, FINDING POINTS OF INTEREST\n\n\n";

	/* Flooding predefined rooms to find their centers */
	for (int x = map_x_min; x < map_x_max; x++) {
		for (int y = map_y_min; y < map_y_max; y++) {
			if (map_matrix[x][y] == true) {
				flood_data.x_total = 0;
				flood_data.y_total = 0;
				flood_data.count = 0;
				floodArea(x, y);

				int x_index = flood_data.x_total/flood_data.count, y_index = flood_data.y_total/flood_data.count;
				float x_world = (x_index-(width / 2.0)) * meters_per_pixel, y_world = (y_index - (height / 2.0)) * meters_per_pixel;
				
				cout << "CENTER AT POINT(" << x_world << ", " << y_world  << ")\n";

				target_points.push_back(x_world);
				target_points.push_back(y_world);
			}
		}
	}

	/* Visualizing Points of Interest */
	// for (int i = 0; i < target_points.size(); i += 2) {
	// 	map_matrix[(target_points[i])][(target_points[i+1])] = true;
	// 	map_matrix[(target_points[i])+1][(target_points[i+1])] = true;
	// 	map_matrix[(target_points[i])-1][(target_points[i+1])] = true;
	// 	map_matrix[(target_points[i])][(target_points[i+1])+1] = true;
	// 	map_matrix[(target_points[i])][(target_points[i+1])-1] = true;
	// }

	//make sure to garbage collect matrices here!
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

float** rotateIterator(float** input, int w, int h) {
	/* Rotating input matrix by 90 degrees */
	float** output = new float*[h];
	for (int i = 0; i < h; i++) output[i] = new float[w];

	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			output[j][w-1-i] = input[i][j];
		}
	}
	return output;
}

void floodArea(int x, int y) {
	/* Recursively flooding and filling valid spots */
	if (map_matrix[x][y] == true) {
		map_matrix[x][y] = false;
		flood_data.x_total += x;
		flood_data.y_total += y;
		flood_data.count++;

		floodArea(x+1, y);
		floodArea(x-1, y);
		floodArea(x, y+1);
		floodArea(x, y-1);
	}
}

int main(int argc, char **argv)
{
	/* Init Garbage */
	ros::init(argc, argv, "preprocess");
	ros::NodeHandle nh;

	/* Subscribe to Map & Metadata */
	ros::Subscriber subscribe_map = nh.subscribe("/map", 1000, &getMap);
	
	/* Requesting Map once */
	while (has_been_preprocessed == false) {
		ros::Duration(.01).sleep();	

		ros::spinOnce();
	}
}
