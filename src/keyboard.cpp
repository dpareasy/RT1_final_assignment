#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "final_assignment/Keyboard.h"
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

using namespace std;

ros::Publisher pub; 
geometry_msgs::Twist my_vel;

void StopRobot()
{
	my_vel.linear.x = 0;
	my_vel.angular.z = 0;
	pub.publish(my_vel);
	cout<<"Set velocities equal to zero"<<endl;
}


int main(int argc, char **argv)
{
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_node");
	ros::NodeHandle nh;
	cout<<"Activating Keyboard navigation modality"<<endl;
	//define the publisher to send message for the velocity of the robot
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//run teleop_twist_keyboard node
	system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");
	
	StopRobot();
	ros::spin();
	return 0;
}


