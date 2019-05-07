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
bool **map_matrix;
bool **new_map_matrix;

/* Iterator to brute-force scan for objects */
float **table_iterator;
int table_iterator_width, table_iterator_height;

float **box_iterator;
int box_iterator_width, box_iterator_height;

/* Bounds for relevant map data [ignoring excessive padding on edges of map] */
int map_x_min, map_x_max, map_y_min, map_y_max;
int new_map_x_min, new_map_x_max, new_map_y_min, new_map_y_max;

/* Sets of two, target x, target y for objects */
vector<float> table_target_points;
vector<float> box_target_points;

/* Smoothing Matrix of erronous/noisy points */
void boxFilter();
void newBoxFilter();

/* Writing Matrix to file for viewing */
void plotMatrix();
void plotNewMatrix();

/* Rotating Iterator Matrix 90 degrees */
float **rotateIterator(float **input, int w, int h);

/* On receiving New Occupancy Grid... */
void getNewMap(const nav_msgs::OccupancyGrid &map)
{
	new_map_has_been_received = true;
	cout << "\nRECEIVED NEW MAP...\n\n\n";

	/* Reading Metadata */
	meters_per_pixel = map.info.resolution;
	width = map.info.width;
	height = map.info.height;

	/* Defining Relevant Boundaries (given 20m x 20m space) */
	new_map_x_min = (width / 2) - (10 / meters_per_pixel);
	new_map_x_max = (width / 2) + (10 / meters_per_pixel);

	new_map_y_min = (height / 2) - (10 / meters_per_pixel);
	new_map_y_max = (height / 2) + (10 / meters_per_pixel);

	cout << new_map_x_min << ", " << new_map_x_max << "\n";
	cout << new_map_y_min << ", " << new_map_y_max << "\n";

	/* Initializing && Populating Map Matrix */
	new_map_matrix = new bool *[height];
	for (int i = 0; i < height; i++)
		new_map_matrix[i] = new bool[width];
	for (int i = 0; i < width * height; i++)
	{
		//if (map.data[i] != 0 ) cout << "yes\n";
		new_map_matrix[i / width][i % width] = map.data[i] == 0;
	}

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

	cout << map_x_min << ", " << map_x_max << "\n";
	cout << map_y_min << ", " << map_y_max << "\n";

	/* Initializing && Populating Map Matrix */
	map_matrix = new bool *[height];
	for (int i = 0; i < height; i++)
		map_matrix[i] = new bool[width];
	for (int i = 0; i < width * height; i++)
		map_matrix[i / width][i % width] = map.data[i] == 0;

	boxFilter();
	plotMatrix();
}

void plotNewMatrix()
{
	/* Writing to file */
	ofstream myfile;
	myfile.open("new_map.txt");
	if (myfile.is_open())
	{
		for (int x = new_map_x_min; x < new_map_x_max; x++)
		{
			for (int y = new_map_y_min; y < new_map_y_max; y++)
			{
				myfile << new_map_matrix[x][y] << " ";
			}
			myfile << "\n";
		}
	}
	else
		cout << "UNABLE TO OPEN PLOT.TXT\n";
	myfile.close();
}
void plotMatrix()
{
	/* Writing to file */
	ofstream myfile;
	myfile.open("map.txt");
	if (myfile.is_open())
	{
		for (int x = map_x_min; x < map_x_max; x++)
		{
			for (int y = map_y_min; y < map_y_max; y++)
			{
				myfile << map_matrix[x][y] << " ";
			}
			myfile << "\n";
		}
	}
	else
		cout << "UNABLE TO OPEN PLOT.TXT\n";
	myfile.close();
}

void boxFilter()
{
	/* If 5 or more neighbors of any pixel are a given value, set pixel to value */
	for (int num_passes = 0; num_passes < 1; num_passes++)
	{
		for (int x = map_x_min; x < map_x_max; x++)
		{
			for (int y = map_y_min; y < map_y_max; y++)
			{
				int num_available_space = 0;
				if (map_matrix[x + 1][y + 0])
					num_available_space++;
				if (map_matrix[x + 0][y + 1])
					num_available_space++;
				if (map_matrix[x + 1][y + 1])
					num_available_space++;
				if (map_matrix[x - 1][y - 0])
					num_available_space++;
				if (map_matrix[x - 0][y - 1])
					num_available_space++;
				if (map_matrix[x - 1][y - 1])
					num_available_space++;
				if (map_matrix[x + 1][y - 1])
					num_available_space++;
				if (map_matrix[x - 1][y + 1])
					num_available_space++;
				if (num_available_space >= 5)
					map_matrix[x + 0][y + 0] = true;
				if (num_available_space <= 3)
					map_matrix[x + 0][y + 0] = false;
			}
		}
	}
}

void newBoxFilter()
{
	/* If 5 or more neighbors of any pixel are a given value, set pixel to value */
	for (int num_passes = 0; num_passes < 1; num_passes++)
	{
		for (int x = map_x_min; x < map_x_max; x++)
		{
			for (int y = map_y_min; y < map_y_max; y++)
			{
				int num_available_space = 0;
				if (new_map_matrix[x + 1][y + 0])
					num_available_space++;
				if (new_map_matrix[x + 0][y + 1])
					num_available_space++;
				if (new_map_matrix[x + 1][y + 1])
					num_available_space++;
				if (new_map_matrix[x - 1][y - 0])
					num_available_space++;
				if (new_map_matrix[x - 0][y - 1])
					num_available_space++;
				if (new_map_matrix[x - 1][y - 1])
					num_available_space++;
				if (new_map_matrix[x + 1][y - 1])
					num_available_space++;
				if (new_map_matrix[x - 1][y + 1])
					num_available_space++;
				if (num_available_space >= 5)
					new_map_matrix[x + 0][y + 0] = true;
				if (num_available_space <= 3)
					new_map_matrix[x + 0][y + 0] = false;
			}
		}
	}
}

void postProcess()
{
	cout << "running\n\n";

	newBoxFilter();

	for (int x = 0; x < map_x_max - map_x_min; x++)
	{
		for (int y = 0; y < map_y_max - map_y_min; y++)
		{
			if (map_matrix[x + map_x_min][y + map_y_min] == 0)
			{
				for (int x_1 = -1; x_1 < 2; x_1++)
				{
					for (int y_1 = -1; y_1 < 2; y_1++)
					{
						new_map_matrix[x + x_1 + new_map_x_min][y + y_1 + new_map_y_min] = 1;
					}
				}
			}
		}
	}

	newBoxFilter();
	plotNewMatrix();

	table_iterator_height = 30;
	table_iterator_width = 40;

	int table_leg_size = 4;

	table_iterator = new float *[table_iterator_width];
	for (int i = 0; i < table_iterator_width; i++)
		table_iterator[i] = new float[table_iterator_height];

	for (int x = 0; x < table_iterator_width; x++)
	{
		for (int y = 0; y < table_iterator_height; y++)
		{
			table_iterator[x][y] = 1;
		}
	}

	for (int y = 0; y < table_leg_size; y++)
	{
		for (int x = 0; x < table_leg_size; x++)
		{
			table_iterator[0 + x][0 + y] = .5;

			table_iterator[table_iterator_width - x - 1][0 + y] = .5;

			table_iterator[0 + x][table_iterator_height - y - 1] = .5;

			table_iterator[table_iterator_width - x - 1][table_iterator_height - y - 1] = .5;
		}
	}

	/* Search for matches against iterator*/
	for (int rotations = 0; rotations < 2; rotations++)
	{
		for (int x = new_map_x_min; x < new_map_x_max - table_iterator_width; x++)
		{
			for (int y = new_map_y_min; y < new_map_y_max - table_iterator_height; y++)
			{
				bool is_voided = false;

				int false_positives = 0;
				int positives = 0;
				/* Checking if point satisfies condition of Iterator*/
				for (int x_it = 0; x_it < table_iterator_width && is_voided == false; x_it++)
				{
					for (int y_it = 0; y_it < table_iterator_height && is_voided == false; y_it++)
					{
						if (new_map_matrix[x + x_it][y + y_it] == 0 && table_iterator[x_it][y_it] == 1)
						{
							is_voided = true;
						}
						if (new_map_matrix[x + x_it][y + y_it] == 1 && table_iterator[x_it][y_it] == .5)
						{
							false_positives++;
						}
						if (new_map_matrix[x + x_it][y + y_it] == 0 && table_iterator[x_it][y_it] == .5)
						{
							positives++;
						}
					}
				}

				if (positives > 20 && is_voided == false)
				{
					float x_world = (x - (width / 2.0)) * meters_per_pixel, y_world = (y - (height / 2.0)) * meters_per_pixel;

					bool is_unique = true;
					for (std::vector<int>::size_type i = 0; i != table_target_points.size(); i += 2)
					{
						if (abs(table_target_points[i] - x_world) > .5 && abs(table_target_points[i + 1] - y_world) > .5)
						{
						}
						else
						{
							is_unique = false;
							cout << "Duplicate Found\n";
						}
					}
					if (is_unique)
					{
						table_target_points.push_back(x_world);
						table_target_points.push_back(y_world);
						cout << "FOUND TABLE AT " << x_world << ", " << y_world << "\n";
					}
				}
			}
		}
		/* Rotating Iterator 90 degrees */
		table_iterator = rotateIterator(table_iterator, table_iterator_width, table_iterator_height);
		int temp = table_iterator_height;
		table_iterator_height = table_iterator_width;
		table_iterator_width = temp;
	}

	/*----------------------------------------------------*/

	box_iterator_height = 10;
	box_iterator_width = 10;

	/* Search for matches against iterator*/
	for (int x = new_map_x_min; x < new_map_x_max - box_iterator_width; x++)
	{
		for (int y = new_map_y_min; y < new_map_y_max - box_iterator_height; y++)
		{
			bool is_voided = false;

			/* Checking if point satisfies condition of Iterator*/
			for (int x_it = 0; x_it < box_iterator_width && is_voided == false; x_it++)
			{
				for (int y_it = 0; y_it < box_iterator_height && is_voided == false; y_it++)
				{
					if (new_map_matrix[x + x_it][y + y_it] == 1)
					{
						is_voided = true;
					}
				}
			}

			if (is_voided == false)
			{
				float x_world = (x - (width / 2.0)) * meters_per_pixel, y_world = (y - (height / 2.0)) * meters_per_pixel;

				bool is_unique = true;
				for (std::vector<int>::size_type i = 0; i != box_target_points.size(); i += 2)
				{
					if (abs(box_target_points[i] - x_world) > .5 && abs(box_target_points[i + 1] - y_world) > .5)
					{
					}
					else
					{
						is_unique = false;
					}
				}
				if (is_unique)
				{
					box_target_points.push_back(x_world);
					box_target_points.push_back(y_world);
					cout << "FOUND BOX AT " << x_world << ", " << y_world << "\n";
				}
			}
		}
	}
}

float **rotateIterator(float **input, int w, int h)
{
	/* Rotating input matrix by 90 degrees */
	float **output = new float *[h];
	for (int i = 0; i < h; i++)
		output[i] = new float[w];

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			output[j][w - 1 - i] = input[i][j];
		}
	}
	return output;
}

int main(int argc, char **argv)
{
	/* Init Garbage */
	ros::init(argc, argv, "postprocess");
	ros::NodeHandle nh;

	/* Subscribe to Map & Metadata */
	ros::Subscriber subscribe_map = nh.subscribe("/map", 1000, &getMap);
	ros::Subscriber subscribe_map_new = nh.subscribe("/myMap", 1000, &getNewMap);

	/* Requesting Map once */
	while (new_map_has_been_received == false || new_map_has_been_received == false)
	{
		ros::Duration(.01).sleep();
		ros::spinOnce();
		cout << "REQUESTING MAPS...\n";
	}

	cout << "RECEIVED BOTH MAPS, PROCESSING RESULTS OF SCAN\n\n\n";

	//........

	postProcess();
}
