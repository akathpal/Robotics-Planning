#include "frontier_turtlebot/TurtlebotFrontier.hpp"
#define OCC_THRESHOLD 10  //  threshold value to determine cell occupancy
#define MAP_OPEN_LIST 1
#define MAP_CLOSE_LIST 2
#define FRONTIER_OPEN_LIST 3
#define FRONTIER_CLOSE_LIST 4

TurtlebotFrontier::TurtlebotFrontier(ros::NodeHandle& nh_){

	pointCloud_publisher = nh_.advertise<sensor_msgs::PointCloud>("frontiers", 1);
	map_subscriber = nh_.subscribe("map", 1, &TurtlebotFrontier::mapCallback, this);
	
}

TurtlebotFrontier::~TurtlebotFrontier(){}

void TurtlebotFrontier::mapCallback(const nav_msgs::OccupancyGrid& map) {

  float resolution = map.info.resolution;
  float map_x = map.info.origin.position.x / resolution;
  float map_y = map.info.origin.position.y / resolution;
  float x = 0. - map_x;
  float y = 0. - map_y;
  std::vector<std::vector<int> > frontiers = this->wfd(map, map.info.height,
      map.info.width, x + (y * map.info.width));
  std::vector<std::vector<int> > map_2d(map.info.height, std::vector<int>(map.info.width, 0));

  for (int i = 0; i < map.info.height; i++) {
    for (int j = 0; j < map.info.width; j++) {
      map_2d[i][j] = (int) map.data[j + i * map.info.width];
    }
  }

  ROS_INFO_STREAM("frontiers size"<< frontiers.size());
  

  
	
	vector<int> frontierMedians;
	for (int i = 0; i < frontiers.size(); i++) {
		int j = getFrontierMedian(frontiers[i]);
		frontierMedians.push_back(j);
    }



  frontier_cloud.points.resize(frontierMedians.size());


  for (int i = 0; i < frontierMedians.size(); i++) {
		frontier_cloud.points[i].x =
		((frontierMedians[i] % map.info.width) + map_x) * resolution;
		frontier_cloud.points[i].y =
		((frontierMedians[i] / map.info.width) + map_y) * resolution;
		frontier_cloud.points[i].z = 0;
    }




  int num_points = 0;
  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {
      num_points++;
    }
  } 

ROS_INFO_STREAM("Num of frontier points "<< num_points);



  for (int i = 0; i < frontiers.size(); i++) {
    for (int j = 0; j < frontiers[i].size(); j++) {

		auto y= (frontiers[i][j]%map.info.width);
		auto x= (frontiers[i][j]/map.info.width);
		map_2d[x][y]  = 5;
    }
  } 


	  for (int i = 0; i < frontierMedians.size(); i++) {
		auto y= (frontierMedians[i]%map.info.width);
		auto x= (frontierMedians[i]/map.info.width);
		map_2d[x][y]  = 10;
    } 
  
/*
  
  FILE *f= fopen("out.ppm", "wb");
  fprintf(f, "P6\n%i %i 255\n", map.info.height,  map.info.width);

    for (int y = 0; y <  map.info.width; y++) {
    for (int x = 0; x < map.info.height; x++) {

      if (map_2d[x][y] == 0) {
        fputc(0, f);
        fputc(0, f);
        fputc(255, f);


      } else if (map_2d[x][y] == -1) {
        fputc(255, f);
        fputc(255, f);
        fputc(255, f);
      }

      else if (map_2d[x][y] == 100) {
        fputc(255, f);
        fputc(0, f);
        fputc(0, f);
      }
      else if(map_2d[x][y] == 5){
      	fputc(0, f);
        fputc(255, f);
        fputc(0, f);
      } else if(map_2d[x][y] == 10){
      	fputc(255, f);
        fputc(255, f);
        fputc(0, f);
      }

    }
  }
  fclose(f); 
  
  system("xdg-open out.ppm"); */
  //ROS_ERROR_STREAM("End of Map Callback");

  pointCloud_publisher.publish(frontier_cloud);
  this->frontier(frontier_cloud);
}


std::vector<std::vector<int> > TurtlebotFrontier::wfd(const nav_msgs::OccupancyGrid& map, int map_height, int map_width, int pose) {	
	
	std::vector<std::vector<int> > frontiers;
	int map_size = map_height * map_width;
	std::map<int, int> cell_states;
	std::queue<int> q_m;
	q_m.push(pose);
	cell_states[pose] = MAP_OPEN_LIST;
	int adj_vector[8];
	int v_neighbours[8];
	//
	//ROS_INFO("wfd 1");
	while(!q_m.empty()) {
		//ROS_INFO("wfd 2");
		int cur_pos = q_m.front();
		q_m.pop();
		//ROS_INFO("cur_pos: %d, cell_state: %d",cur_pos, cell_states[cur_pos]);
		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
			continue;
		if(this->is_frontier_point(map, cur_pos, map_size, map_width)) {
			queue<int> q_f;
			vector<int> new_frontier;
			q_f.push(cur_pos);
			cell_states[cur_pos] = FRONTIER_OPEN_LIST;
			// Second BFS
			while(!q_f.empty()) {
				//ROS_INFO("wfd 3");
				//ROS_INFO("Size: %d", q_f.size());
				int n_cell = q_f.front();
				q_f.pop();
				//
				if(cell_states[n_cell] == MAP_CLOSE_LIST || cell_states[n_cell] == FRONTIER_CLOSE_LIST)
					continue;
				//
				if(this->is_frontier_point(map, n_cell, map_size, map_width)) {
					//ROS_INFO("adding %d to frontiers", n_cell);
					new_frontier.push_back(n_cell);
					this->get_neighbours(adj_vector, n_cell, map_width);			
					//
					//ROS_INFO("wfd 3.5");
					for(int i = 0; i < 8; i++) {
						if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
							if(cell_states[adj_vector[i]] != FRONTIER_OPEN_LIST && 
								cell_states[adj_vector[i]] != FRONTIER_CLOSE_LIST && 
								cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
								//ROS_INFO("wfd 4");
								if(map.data[adj_vector[i]] != 100) {
									q_f.push(adj_vector[i]);
									cell_states[adj_vector[i]] = FRONTIER_OPEN_LIST;
								}
							}
						}
					}
				}
				cell_states[n_cell] = FRONTIER_CLOSE_LIST;
			}
			if(new_frontier.size() > 5)
				frontiers.push_back(new_frontier);
			
			//ROS_INFO("WFD 4.5");
			for(unsigned int i = 0; i < new_frontier.size(); i++) {
				cell_states[new_frontier[i]] = MAP_CLOSE_LIST;
				//ROS_INFO("WFD 5");
			}
		}
		//
		this->get_neighbours(adj_vector, cur_pos, map_width);

		for (int i = 0; i < 8; ++i) {
			//ROS_INFO("wfd 6");
			if(adj_vector[i] < map_size && adj_vector[i] >= 0) {
				if(cell_states[adj_vector[i]] != MAP_OPEN_LIST &&  cell_states[adj_vector[i]] != MAP_CLOSE_LIST) {
					this->get_neighbours(v_neighbours, adj_vector[i], map_width);
					bool map_open_neighbor = false;
					for(int j = 0; j < 8; j++) {
						if(v_neighbours[j] < map_size && v_neighbours[j] >= 0) {
							if(map.data[v_neighbours[j]] < OCC_THRESHOLD && map.data[v_neighbours[j]] >= 0) { //>= 0 AANPASSING
								map_open_neighbor = true;
								break;
							}
						}
					}
					if(map_open_neighbor) {
						q_m.push(adj_vector[i]);
						cell_states[adj_vector[i]] = MAP_OPEN_LIST;
					}
				}
			}
		}
		//ROS_INFO("wfd 7");
		cell_states[cur_pos] = MAP_CLOSE_LIST;
		//ROS_INFO("wfd 7.1");
	}
	// ROS_INFO("wfd 8");
	return frontiers;
}

void TurtlebotFrontier::get_neighbours(int n_array[], int position, int map_width) {
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
}

bool TurtlebotFrontier::is_frontier_point(const nav_msgs::OccupancyGrid& map, int point, int map_size, int map_width) {
	const int MIN_FOUND = 1;
	// The point under consideration must be known
	if(map.data[point] != -1) {
		return false;
	}
	//
	int locations[8]; 
	get_neighbours(locations, point, map_width);
	int found = 0;
	for(int i = 0; i < 8; i++) {
		if(locations[i] < map_size && locations[i] >= 0) {
			// None of the neighbours should be occupied space.		
			if(map.data[locations[i]] > OCC_THRESHOLD) {
				return false;
			}
			//At least one of the neighbours is open and known space, hence frontier point
			if(map.data[locations[i]] == 0) {
				found++;
				//
				if(found == MIN_FOUND) 
					return true;
			}
			//}
		}
	}
	return false;
}

int TurtlebotFrontier::get_row_from_offset(int offset, int width) {
	return offset / width;
}

int TurtlebotFrontier::get_column_from_offset(int offset, int width) {
	return offset % width;	
}


void TurtlebotFrontier::frontier(const sensor_msgs::PointCloud frontier_cloud) {

  if (frontier_cloud.points.size() == 0)
    return;
  
  bool at_target = false;
  int frontier_i = this->getNearestFrontier(frontier_cloud);
  ROS_INFO("Closest frontier: %d", frontier_i);

//	if(count>2)
//{ 

//}
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  


    goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
    goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server active");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(20.0));
    ROS_INFO("move_base goal published");
	int count = 0;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || count < 2) {
      at_target = true;
      ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(3.14);
      goal.target_pose.pose.orientation = odom_quat;
      ac.sendGoal(goal);
      ac.waitForResult();
		count++;
	} else {

	 frontier_i = (rand() % frontier_cloud.points.size());
	ROS_WARN_STREAM("Frontier navigation failed, rerouting to farthest frontier");

    goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
    goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server active");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(20.0));
    ROS_INFO("move_base goal published");
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	  ROS_INFO_STREAM("ATTEMPT #: " << count );
      at_target = true;
      ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(3.14);
      goal.target_pose.pose.orientation = odom_quat;
      ac.sendGoal(goal);
      ac.waitForResult();
		count++;
//	}


}


/*
	else {
	ROS_INFO_STREAM("Stuck, trying a random frontier");
  int attempts = 0;

	while (!at_target && attempts < 2) {
    if (attempts > 0) {
      frontier_i = (rand() % frontier_cloud.points.size());
    }
	goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
    goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server active");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(45.0));
    ROS_INFO("move_base goal published");
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      at_target = true;
      ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(3.14);
      goal.target_pose.pose.orientation = odom_quat;
      ac.sendGoal(goal);
      ac.waitForResult();
    attempts++; 
	} */
    }
  }
//}


int TurtlebotFrontier::getNearestFrontier(const sensor_msgs::PointCloud frontier_cloud) {
  
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0), map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 100000;
  for (int i = 0; i < frontier_cloud.points.size(); i++) {
    float distance = this->getDistance(frontier_cloud.points[i].x,
                                 map_transform.getOrigin().x(),
                                 frontier_cloud.points[i].y,
                                 map_transform.getOrigin().y());
    if (distance > .7 && distance <= closest_frontier_distance) {
      closest_frontier_distance = distance;
      frontier_i = i;
    }
  }
  return frontier_i;
}

int TurtlebotFrontier::getFarthestFrontier(const sensor_msgs::PointCloud frontier_cloud) {
  
  tf::TransformListener map_listener;
  tf::StampedTransform map_transform;
  map_listener.waitForTransform("/map", "/base_link",
                               ros::Time(0), ros::Duration(3.0));
  map_listener.lookupTransform("/map", "/base_link", ros::Time(0), map_transform);
  int frontier_i = 0;
  float closest_frontier_distance = 0;
  for (int i = 0; i < frontier_cloud.points.size(); i++) {
    float distance = this->getDistance(frontier_cloud.points[i].x,
                                 map_transform.getOrigin().x(),
                                 frontier_cloud.points[i].y,
                                 map_transform.getOrigin().y());
    if (distance >= closest_frontier_distance) {
      closest_frontier_distance = distance;
      frontier_i = i;
    }
  }
  return frontier_i;
}


float TurtlebotFrontier::getDistance(float x1, float x2, float y1, float y2) {
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

int TurtlebotFrontier::getFrontierMedian(vector<int> frontiers_) {    
  sort(frontiers_.begin(), frontiers_.end());
  return frontiers_[frontiers_.size()/2];
}
