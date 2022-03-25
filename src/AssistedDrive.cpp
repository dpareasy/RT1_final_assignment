#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

//defining the dimension of each array for visual ranges
#define RIGHT_DIM 40
#define FRONT_RIGHT_DIM 200
#define FRONT_DIM 240
#define FRONT_LEFT_DIM 200
#define LEFT_DIM 40

//arrays for visual ranges
double right_dist[RIGHT_DIM];
double front_right_dist[FRONT_RIGHT_DIM];
double front_dist[FRONT_DIM];
double front_left_dist[FRONT_LEFT_DIM];
double left_dist[LEFT_DIM];

//minimum distances
double min_right;
double min_left;
double min_front;

//defining a threashold
double th = 1;

//initialize the value of the velocities
double linearV;
double angularV;

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Callback function
ros::Publisher pub; 

//defining my_vel variable
geometry_msgs::Twist my_vel;

//function to get the robot velocity and save in a variable
void Velocity(const geometry_msgs::Twist::ConstPtr& V){

	linearV = V -> linear.x;
	angularV = V -> angular.z;
}

//function that allows to 
void AssistedNavigation(const sensor_msgs::LaserScan::ConstPtr& msg){

	//Here each array for visual ranges is created. I decided to divide the visual field of the robot in 5 sections
	//but for make it avoid obstacles front_right_distance and front_left_distance are not used.
	for (int i = 0; i < msg->ranges.size(); i++){
	
		//The array for the right visual field. It is between an agle of 170 and 180 degrees.
		for(int h = 0; h < RIGHT_DIM; h++){
		
			right_dist[h] = msg->ranges[i++];
		}
		//The array for the front-right visual field. It is between an agle of 110 and 170 degrees.
		for(int j = 0; j < FRONT_RIGHT_DIM; j++){
		
			front_right_dist[j] = msg->ranges[i++];
		}
		//The array for the frontal visual field. It is between an agle of 70 and 110 degrees.
		for(int k = 0; k < FRONT_DIM; k++){
		
			front_dist[k] = msg->ranges[i++];
		}
		//The array for the front-left visual field. It is between an agle of 10 and 70 degrees.
		for(int l = 0; l < FRONT_LEFT_DIM; l++){
		
			front_left_dist[l] = msg->ranges[i++];
		}
		//The array for the left visual field. It is between an agle of 0 and 10 degrees.
		for(int n = 0; n < LEFT_DIM; n++){
		
			left_dist[n] = msg->ranges[i++];
		}
	}
	//To know the direction in which to move the robot,
	//it is required to know the minimum distance from the wall in each field.
	//To do this it is simply needed to check the minimum value for each array.
	min_right = right_dist[0];
	for(int i=0; i<RIGHT_DIM; i++){
	
		if(min_right > right_dist[i])
			min_right = right_dist[i];
	}
	
	min_front = front_dist[0];
	for(int i=0; i<FRONT_DIM; i++){
	
		if(min_front > front_dist[i])
			min_front = front_dist[i];
	}
		
	min_left = left_dist[0];
	for(int i=0; i<LEFT_DIM; i++){
	
		if(min_left > left_dist[i])
			min_left = left_dist[i];
	}
	
	my_vel.linear.x = linearV;
	my_vel.angular.z = angularV;
	
	if (min_front <= th || min_left <= th || min_right <=th){

		//if right distance is greater than left distance
		//make the robot turn right
		if(min_right > min_left){
		
			my_vel.linear.x = 0;
			my_vel.angular.z = -1;
		}
		//if left distance is greater than right distance
		//make the robot turn left
		if(min_right < min_left){
		
			my_vel.linear.x = 0;
			my_vel.angular.z = 1;
		}
				
	}
	pub.publish(my_vel);
}


int main(int argc, char **argv){

	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_avoidcoll_node");
	ros::NodeHandle nh;
	
	//publisher to publish the velocities on the topic
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//subscriber for getting the robot velocity 
	ros::Subscriber subV = nh.subscribe("/cmd_vel_assisted", 1, Velocity);
	
	//define the subscriber to compute the distance from walls
	//the topic in which the other nodes use to publish their position
	ros::Subscriber sub = nh.subscribe("/scan", 1, AssistedNavigation); 
	
	ros::spin();
	return 0;
}


