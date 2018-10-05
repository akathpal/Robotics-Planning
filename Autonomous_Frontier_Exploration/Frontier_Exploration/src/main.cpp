#include "frontier_turtlebot/TurtlebotFrontier.hpp"

int main(int argc, char **argv){

	ROS_INFO_STREAM("Main begins");

	ros::init(argc, argv, "frontier_detection");  //  Initiate new ROS node

	ros::NodeHandle nh_;

	geometry_msgs::Twist rotate;

	ros::Publisher rotate_pub = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

	int count = 0;
	ros::Rate loop_rate(2);
	while (nh_.ok() && count < 100) {
    ROS_INFO_STREAM_ONCE("ROTATING INITIALLY FOR 10 SECONDS");
    rotate.linear.x = 0;
    rotate.angular.z = 0.2;
		
    rotate_pub.publish(rotate);
    ros::spinOnce();
    loop_rate.sleep();
	count ++;
  }

    rotate.linear.x = 0;
    rotate.angular.z = 0.0;
	rotate_pub.publish(rotate);
    ros::spinOnce();
    loop_rate.sleep();


	

	ROS_INFO_STREAM("Creating Turtlebot Frontier Object");
	TurtlebotFrontier frontier_exploration(nh_);
	ROS_INFO("Frontiers Detected");
	ros::Duration(1.0).sleep();
	
	//For testing infinite stuck
	ROS_INFO("Going to stuck");
	ros::spin();
	return 0;
}
