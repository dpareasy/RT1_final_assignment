#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
//#include "final_assignment/Keyboard.h"
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

void NavigationCommands()
{
	cout <<"Enter a choice and press ENTER to move the robot:"<<endl;
	cout <<"A---rotate right"<<endl;
	cout <<"D---rotate left"<<endl;
	cout <<"W---move traightforward"<<endl;
	cout <<"S---move backward"<<endl;
	cout <<"E---move front left"<<endl;
	cout <<"Q---move front right"<<endl;
	cout <<"X---stop the robot"<<endl;
	cout <<"R---EXIT the program"<<endl;
	
}

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
				my_vel.linear.x = 0;
				my_vel.angular.z = -1;
				break;
				
			
			case 'D':
			case 'd':
				my_vel.linear.x = 0;
				my_vel.angular.z = 1;
				break;
			
			case 'W':
			case 'w':
				my_vel.linear.x = 0.5;
				my_vel.angular.z = 0;
				break;
			
			case 'S':
			case 's':
				my_vel.linear.x = -0.5;
				my_vel.angular.z = 0;
				break;
			
			case 'E':
			case 'e':
				my_vel.linear.x = 0.5;
				my_vel.angular.z = -1;
				break;
			
			case 'Q':
			case 'q':
				my_vel.linear.x = 0.5;
				my_vel.angular.z = 1;
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
		pub.publish(my_vel);
	}
}

int main(int argc, char **argv)
{
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "teleopkey_node");
	ros::NodeHandle nh;
	cout<<"Activating Keyboard navigation modality"<<endl;
	//define the publisher to send message for the velocity of the robot
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	Navigation();
	ros::spin();
	system("clear");
	return 0;
}


