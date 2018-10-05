
#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include "ros/console.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "ros/ros.h"
#include <cstdlib> // Needed for rand()
#include <ctime>
#include "nav_msgs/OccupancyGrid.h"
using namespace std;

class TurtlebotFrontier{
public:
	TurtlebotFrontier(ros::NodeHandle& nh_);
    ~TurtlebotFrontier();
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void get_neighbours(int n_array[], int position, int map_width);
	nav_msgs::OccupancyGrid downSizeMap(const nav_msgs::OccupancyGrid& map, int width, int height);
	bool is_frontier_point(const nav_msgs::OccupancyGrid& map, int point, int map_size, int map_width);
	int get_row_from_offset(int offset, int width);
	int get_column_from_offset(int offset, int width);
	std::vector<std::vector<int> > wfd(const nav_msgs::OccupancyGrid& map, int map_height, int map_width, int pose);
	void frontier(const sensor_msgs::PointCloud frontier_cloud);
	int getNearestFrontier(const sensor_msgs::PointCloud frontier_cloud);
	float getDistance(float x1, float x2, float y1, float y2);
	int getFrontierMedian(vector<int>);
	int getFarthestFrontier(const sensor_msgs::PointCloud);
	
private:
	sensor_msgs::PointCloud frontier_cloud;
	ros::Subscriber map_subscriber;
	ros::Publisher pointCloud_publisher;

};
