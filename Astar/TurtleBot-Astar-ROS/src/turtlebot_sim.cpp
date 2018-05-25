#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <string>

using namespace std;

geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;


/* function to obtain current pose of turtlebot */
void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  // linear position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;

  // quaternion to RPY conversion
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // angular position
  current_pose.theta = yaw;
  pub_pose2d.publish(current_pose);

}



int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_sim");
  ros::NodeHandle n;
  // sleep for 20 seconds while gazebo starts
  ros::Duration(5).sleep();
  ros::Subscriber subLaserScan;
  ros::Publisher pub;
  geometry_msgs::Twist msg;
  ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  pub_pose2d = n.advertise<geometry_msgs::Pose2D>("turtlebot_pose2d", 1);

  ros::Subscriber sub_odometry = n.subscribe("odom", 1, odomCallback);

  pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
                                          100);

  ofstream Velocities;
  Velocities.open("Velocities.txt");

		ifstream ip("/home/abhishek/my_catkin_ws/src/turtlebot_sim/filename.csv");
	if(!ip.is_open()){
			cout << "error" << endl;
			return 0;
		}
		string v;
		string w;


  const double PI = 3.14159265358979323846;
  ros::Rate loop_rate(1);

	/* Rotate 45 */
  while (n.ok() && ip.good()) {
    ros::spinOnce();
		getline(ip, v, ',');
		getline(ip, w, '\n');
//    ROS_INFO_STREAM_ONCE("ROTATING");
    msg.linear.x = std::stod(v);
    msg.angular.z = std::stod(w);
    ROS_INFO_STREAM(
        "x: " << "linear.x = " << msg.linear.x << "angular.z = " << msg.angular.z);
		Velocities << "linear.x = " << msg.linear.x <<", linear.y = "<< msg.linear.y <<", linear.z =  0.0" << ", angular.x = 0.0" <<", angular.y = 0.0" << ", angular.z = " << msg.angular.z <<endl;
		
    pub.publish(msg);
    loop_rate.sleep();
    // ros::Duration(1).sleep();
  }

	



  return 0;

//    pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
}

