/**
*\file AssistedDrive.cpp
*\brief Collision Avoidance
*\author Parisi Davide Leo S4329668
*\version 1.0
*\date 08/04/2022
*\details
*
*Subscribes to: <BR>
* /cmd_vel_assisted
* /scan
*Publishes to: <BR>
* /cmd_vel
*
*Description:
*
*This node simulate the assisted nvigation of a robt within the environment. It asks the user to drive with the keyboard.
*Once the robot detect a wall in the vicinity, a system of assisted drive will make the robot avoid collisions by adjusting the trajctory.
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

//defining the dimension of each 
//array for visual ranges
#define RIGHT_DIM 80
#define FRONT_RIGHT_DIM 240
#define FRONT_DIM 80
#define FRONT_LEFT_DIM 240
#define LEFT_DIM 80

double right_dist[RIGHT_DIM]; ///< Array for right visual range
double front_right_dist[FRONT_RIGHT_DIM]; ///< Array for front-right visual range
double front_dist[FRONT_DIM]; ///< Array for frontal visual range
double front_left_dist[FRONT_LEFT_DIM]; ///< Array for front left visual range
double left_dist[LEFT_DIM]; ///< Array for left visual range

double min_right; ///< variable for minimum value of right visual range
double min_left; ///< variable for minimum value of left visual range
double min_front; ///< variable for minimum value of front visual range

//NOT USED BUT CAN BE USEFUL 
//FOR POSSIBLE IMPROVEMENTS
//double min_front_left;
//double min_front_right;

double th = 0.8; ///< defining a threashold 

//initialize the value of the velocities
double linearV; ///< variable for the linear velocity
double angularV; ///< variable for the angular velocity

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Callback function
ros::Publisher pubV; ///< The publisher to publish velocity

//defining my_vel variable
geometry_msgs::Twist my_vel; ///< The variable to 


/**
*\brief Description of Quit() function:
*
*The aim of this function is to give the user the 
*possibility to exit the modality.
**/
void Quit(){

	for(;;){
		system("clear");
		
		char quit;
		cin >> quit;
		switch(quit){
		
			case 'Q':
			case 'q':
				my_vel.linear.x = 0;
				my_vel.angular.z = 0;
				pubV.publish(my_vel);
				exit (0);
				break;
			default:
				cout<<"Invalid input"<<endl;
				break;
		}
	}
}
/**
*\brief Description of GetVelocity() function:
*
*\param V it's a pointer to the message published on /cmd_vel_assisted topic.
*
*The aim of this function is to get the velocity of the robot
*from the teleop_twist_keyboard and save them in two variables.
*One for the linear velocity and the other for the angular velocity.
**/
void GetVelocity(const geometry_msgs::Twist::ConstPtr& V){

	linearV = V -> linear.x;
	angularV = V -> angular.z;
}


/**
*\brief Description of AssistedNavigation() function:
*
*\param msg is a pointer to the message published on /scan topic.
*
*In this function each array for visual ranges is created.
*I decided to divide the visual field of the robot in 5 sections.
*But for making it avoid obstacles two of them aren't considered
*in this envinronments. They can be used for possible improvements.
*This function has the aim of detecting when the robot gets too close 
*to the walls under a prefixed threshol. When this happens, the user is 
*notified and the robot will adjusts its trajectory or by its own or with 
*the help of the user.
**/
void AssistedNavigation(const sensor_msgs::LaserScan::ConstPtr& msg){

	//Here each array for visual ranges is created. 
	//I decided to divide the visual field of the robot in 5 sections
	//but for make it avoid obstacles front_right_distance 
	//and front_left_distance are not used in this environment.
	//They can be used for possible improvements.
	for (int i = 0; i < msg->ranges.size(); i++){
	
		//The array for the right visual field. 
		for(int h = 0; h < RIGHT_DIM; h++){
		
			right_dist[h] = msg->ranges[i++];
		}
		//The array for the front-right visual field. 
		for(int j = 0; j < FRONT_RIGHT_DIM; j++){
		
			front_right_dist[j] = msg->ranges[i++];
		}
		//The array for the frontal visual field. 
		for(int k = 0; k < FRONT_DIM; k++){
		
			front_dist[k] = msg->ranges[i++];
		}
		//The array for the front-left visual field. 
		for(int l = 0; l < FRONT_LEFT_DIM; l++){
		
			front_left_dist[l] = msg->ranges[i++];
		}
		//The array for the left visual field.
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
	
	/* USEFUL FOR POSSIBLE IMPROVEMENT IN THE DETECTION OF THE WALLS
	min_front_right = front_right_dist[0];
	for(int i=0; i<RIGHT_DIM; i++){
	
		if(min_front_right > front_right_dist[i])
			min_front_right = front_right_dist[i];
	}
	*/
	
	min_front = front_dist[0];
	for(int i=0; i<FRONT_DIM; i++){
	
		if(min_front > front_dist[i])
			min_front = front_dist[i];
	}
	
	/* USEFUL FOR POSSIBLE IMPROVEMENT IN THE DETECTION OF THE WALLS
	min_front_left = front_left_dist[0];
	for(int i=0; i<LEFT_DIM; i++){
	
		if(min_front_left > front_left_dist[i])
			min_front_left = front_left_dist[i];
	}
	*/
		
	min_left = left_dist[0];
	for(int i=0; i<LEFT_DIM; i++){
	
		if(min_left > left_dist[i])
			min_left = left_dist[i];
	}
	
	//updating linear and angular velocity
	my_vel.linear.x = linearV;
	my_vel.angular.z = angularV;
	
	//if a wall detected on the front.
	//readjust trajectory
	if (min_front <= th){
	
		system("clear");
		cout<<"type 'Q' on this konsole if you want to change modality\n\n\n\n\n\n\n\n\n\n\n\n\n"<<endl;
		
		//warn the user
		cout<<"!!WARNING!!\nWall detected in front: tuorn either left or right"<<endl;
		
		//set linear velocity equal to zero
		my_vel.linear.x = 0;
	}
	//if a wall detected on the left.
	//readjust trajectory  
	else if (min_left <= th){
			
			system("clear");
			cout<<"type 'Q' on this konsole if you want to change modality\n\n\n\n\n\n\n\n\n\n\n\n\n"<<endl;
			//ask the user to turn right
			cout <<"!!WARNING!!\nWall detected on the left, adjusting trajectory!"<<endl;
			
			//set angular velocity equal to zero
			my_vel.angular.z = -1;
			
		
	}
	//if a wall detected on the right
	else if (min_right <= th){
	
			system("clear");
			cout<<"type 'Q' on this konsole if you want to change modality\n\n\n\n\n\n\n\n\n\n\n\n\n"<<endl;
			//ask the user tu turn left
			cout <<"!!WARNING!!\nWall detected on the right, adjusting trajectory!"<<endl;		
			
			//set nagular velocity equal to zero
			my_vel.angular.z = 1;
			
	}
	else{		
		system("clear");
		cout<<"type 'Q' on this konsole if you want to change modality\n\n\n\n\n\n\n\n\n\n\n\n\n"<<endl;
		
		//No wall detected
		cout<<"No walls in the vicinity!\nDrive the robot with the keyboard in the other konsole!"<<endl;
	}
	//publishing the value of the velocities		
	pubV.publish(my_vel);
}


int main(int argc, char **argv){

	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_avoidcoll_node");
	ros::NodeHandle nh;
	
	//publisher to publish the velocities on the topic
	pubV = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//subscriber for getting the robot velocity 
	ros::Subscriber subV = nh.subscribe("/cmd_vel_assisted", 1, GetVelocity);
	
	//define the subscriber to compute the distance from walls
	//the topic in which the other nodes use to publish their position
	ros::Subscriber sub = nh.subscribe("/scan", 1, AssistedNavigation); 
	
	//multi-threading
	ros::AsyncSpinner spinner(4);
	
	spinner.start();
		Quit();
	spinner.stop();
	return 0;
}


