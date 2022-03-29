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

//NOT USED BUT CAN BE USEFUL 
//FOR POSSIBLE IMPROVEMENTS
//double min_front_left;
//double min_front_right;

//defining a threashold
double th = 0.8;

//initialize the value of the velocities
double linearV;
double angularV;

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Callback function
ros::Publisher pub; 

//defining my_vel variable
geometry_msgs::Twist my_vel;


//Give the user the possibility to exit the node
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
				pub.publish(my_vel);
				exit (0);
				break;
			default:
				cout<<"Invalid input"<<endl;
				break;
		}
	}
}
//function to get the robot velocity and save in a variable
void GetVelocity(const geometry_msgs::Twist::ConstPtr& V){

	linearV = V -> linear.x;
	angularV = V -> angular.z;
}


//function that allows to assist the robot navigation
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
	pub.publish(my_vel);
}


int main(int argc, char **argv){

	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_avoidcoll_node");
	ros::NodeHandle nh;
	
	//publisher to publish the velocities on the topic
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	//subscriber for getting the robot velocity 
	ros::Subscriber subV = nh.subscribe("/cmd_vel_assisted", 1, GetVelocity);
	
	//define the subscriber to compute the distance from walls
	//the topic in which the other nodes use to publish their position
	ros::Subscriber sub = nh.subscribe("/scan", 1, AssistedNavigation); 
	
	ros::AsyncSpinner spinner(4);
	
	spinner.start();
		Quit();
	spinner.stop();
	return 0;
}


