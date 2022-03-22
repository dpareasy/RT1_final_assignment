#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
//#include "final_assignment/Avoidcoll.h"

using namespace std;

//defining the dimension of each array for visual ranges
#define RIGHT_DIM 40
#define FRONT_RIGHT_DIM 240
#define FRONT_DIM 160
#define FRONT_LEFT_DIM 240
#define LEFT_DIM 40
//defining value for maximum angular 
//and linear velocity
#define MAX_LINEAR_VELOCITY 8
#define MAX_ANGULAR_VELOCITY 8
//defining value for minimum angular 
//and linear velocity
#define MIN_LINEAR_VELOCITY 0
#define MIN_ANGULAR_VELOCITY 0

//Here, as global variable are defined the arrays for visual ranges
float right_dist[RIGHT_DIM];
float front_right_dist[FRONT_RIGHT_DIM];
float front_dist[FRONT_DIM];
float front_left_dist[FRONT_LEFT_DIM];
float left_dist[LEFT_DIM];

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Callback function
ros::Publisher pub; 

void AssistedNavigation(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	
	//Here each array for visual ranges is created. I decided to divide the visual field of the robot in 5 sections
	//but for make it avoid obstacles front_right_distance and front_left_distance are not used.
	for (int i = 0; i < msg->ranges.size(); i++) 
	{
		//The array for the right visual field. It is between an agle of 170 and 180 degrees.
		for(int h = 0; h < RIGHT_DIM; h++)
		{
			right_dist[h] = msg->ranges[i++];
		}
		//The array for the front-right visual field. It is between an agle of 110 and 170 degrees.
		for(int j = 0; j < FRONT_RIGHT_DIM; j++)
		{
			front_right_dist[j] = msg->ranges[i++];
		}
		//The array for the frontal visual field. It is between an agle of 70 and 110 degrees.
		for(int k = 0; k < FRONT_DIM; k++)
		{
			front_dist[k] = msg->ranges[i++];
		}
		//The array for the front-left visual field. It is between an agle of 10 and 70 degrees.
		for(int l = 0; l < FRONT_LEFT_DIM; l++)
		{
			front_left_dist[l] = msg->ranges[i++];
		}
		//The array for the left visual field. It is between an agle of 0 and 10 degrees.
		for(int n = 0; n < LEFT_DIM; n++)
		{
			left_dist[n] = msg->ranges[i++];
		}
	}
	//To know the direction in which to move the robot,
	//it is required to know the minimum distance from the wall in each field.
	//To do this it is simply needed to check the minimum value for each array.
	float min_right = right_dist[0];
	for(int i=0; i<RIGHT_DIM; i++)
	{
		if(min_right > right_dist[i])
			min_right = right_dist[i];
	}
	
	float min_front = front_dist[0];
	for(int i=0; i<FRONT_DIM; i++)
	{
		if(min_front > front_dist[i])
			min_front = front_dist[i];
	}
		
	float min_left = left_dist[0];
	for(int i=0; i<LEFT_DIM; i++)
	{
		if(min_left > left_dist[i])
			min_left = left_dist[i];
	}
}


int main(int argc, char **argv)
{
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_avoidcoll_node");
	ros::NodeHandle nh;
	//define the subscriber to compute the distance from walls
	//the topic in which the other nodes use to publish their position 
	ros::Subscriber sub = nh.subscribe("/scan", 1, AssistedNavigation);
	//open the teleop_twist_keyboard
	system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");
	ros::spin();

	return 0;
}


