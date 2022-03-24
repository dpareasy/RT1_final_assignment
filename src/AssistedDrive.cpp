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

#define INITIAL_LINEAR_VELOCITY 0.5
#define INITIAL_ANGULAR_VELOCITY 1

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
double lv = INITIAL_LINEAR_VELOCITY;
double av = INITIAL_ANGULAR_VELOCITY;

//
double linearV;
double angularV;

//defining the input var
char choice;


//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Callback function
ros::Publisher pub; 

//defining my_vel variable
geometry_msgs::Twist my_vel;


//function that makes 
//the robot stop moving
void StopRobot()
{
	linearV = 0;
	angularV = 0;
	cout<<"Set velocities equal to zero"<<endl;
}

//function to get velocities 
//from Navigation()
void RobotVelocity()
{
	my_vel.linear.x = linearV*lv;
	my_vel.angular.z = angularV*av;
	pub.publish(my_vel);
}

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
	min_right = right_dist[0];
	for(int i=0; i<RIGHT_DIM; i++)
	{
		if(min_right > right_dist[i])
			min_right = right_dist[i];
	}
	
	min_front = front_dist[0];
	for(int i=0; i<FRONT_DIM; i++)
	{
		if(min_front > front_dist[i])
			min_front = front_dist[i];
	}
		
	min_left = left_dist[0];
	for(int i=0; i<LEFT_DIM; i++)
	{
		if(min_left > left_dist[i])
			min_left = left_dist[i];
	}
	
	RobotVelocity();
	
	if (min_front < th)
	{

		//if right distance is greater than left distance
		//make the robot turn right
		if(min_right > min_left)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = -1;
		}
		//if left distance is greater than right distance
		//make the robot turn left
		if(min_right < min_left)
		{
			my_vel.linear.x = 0;
			my_vel.angular.z = 1;
		}
				
	}
	pub.publish(my_vel);
}

//menu for using the keyboard navigation
void NavigationCommands()
{
	cout <<"######|Enter a choice|######"<<endl;
	cout <<" A+ENTER rotate right     "<<endl;
	cout <<" D+ENTER rotate left      "<<endl;
	cout <<" W+ENTER move forward     "<<endl;
	cout <<" S+ENTER move backward    "<<endl;
	cout <<" E+ENTER move front left  "<<endl;
	cout <<" Q+ENTER move front right "<<endl;
	cout <<"############################"<<endl;
	cout <<" 1+ENTER to increase linear velocity "<<endl;
	cout <<" 2+ENTER to increase angular velocity"<<endl;
	cout <<" X+ENTER stop the robot"<<endl;
	cout <<" R+ENTER to EXIT the program"<<endl;
	cout <<"############################"<<endl;
}

//navigation states
void Navigation()
{
	for(;;)
	{	
		system("clear");
		NavigationCommands();
		char choice;
		cin >> choice;
		switch(choice)
		{
			case 'A':
			case 'a':
				linearV = 0;
				angularV = 1;
				break;
				
			
			case 'D':
			case 'd':
				linearV = 0;
				angularV = 1;
				break;
			
			case 'W':
			case 'w':
				my_linear = 1;
				angularV = 0;
				break;
			
			case 'S':
			case 's':
				linearV = -1;
				angularV  = 0;
				break;
			
			case 'E':
			case 'e':
				linearV = 1;
				angularV  = -1;
				break;
			
			case 'Q':
			case 'q':
				linearV = 1;
				angularV  = 1;
				break;
			case '1':
				linearV *= 1.2;
				angularV *= 1.2;
				break;
			case '2':
				linearV *= 0.8;
				angularV *= 0.8;
				break;
			case 'X':
			case 'x':
				StopRobot();
				break;
			case 'R':
			case 'r':
				StopRobot();
				system("clear");
				exit(0);
			default:
				cout <<"Invalid input, retry"<<endl;
				break;
		}
		//pub.publish(my_vel);
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
	//publisher
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//ros::AsyncSpinner spinner(4);
    	//spinner.start();
		Navigation();
	//spinner.stop();
	ros::spin();
	return 0;
}


