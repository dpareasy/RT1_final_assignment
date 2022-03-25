#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sys/types.h>
#include <unistd.h>

#define INITIAL_LINEAR_VELOCITY 0.5
#define INITIAL_ANGULAR_VELOCITY 1

using namespace std;

ros::Publisher pub; 
geometry_msgs::Twist my_vel;

//initialize the value of the velocities
float lv = INITIAL_LINEAR_VELOCITY;
float av = INITIAL_ANGULAR_VELOCITY;

void StopRobot(){

	my_vel.linear.x = 0;
	my_vel.angular.z = 0;
	pub.publish(my_vel);
	cout<<"Set velocities equal to zero"<<endl;
}

void NavigationCommands(){

	cout <<"######|Enter a choice|######"<<endl;
	cout <<" A+ENTER rotate right     "<<endl;
	cout <<" D+ENTER rotate left      "<<endl;
	cout <<" W+ENTER move forward     "<<endl;
	cout <<" S+ENTER move backward    "<<endl;
	cout <<" E+ENTER move front right  "<<endl;
	cout <<" Q+ENTER move front left "<<endl;
	cout <<"############################"<<endl;
	cout <<" 1+ENTER to increase linear velocity "<<endl;
	cout <<" 2+ENTER to increase angular velocity"<<endl;
	cout <<" X+ENTER stop the robot"<<endl;
	cout <<" R+ENTER to EXIT the program"<<endl;
	cout <<"############################"<<endl;
}

void Navigation(){

	for(;;){
		
		system("clear");
		NavigationCommands();
		char choice;
		cin >> choice;
		switch(choice){
		
			case 'A':
			case 'a':
				my_vel.linear.x = lv*0;
				my_vel.angular.z = av;
				break;
				
			
			case 'D':
			case 'd':
				my_vel.linear.x = lv*0;
				my_vel.angular.z = -av;
				break;
			
			case 'W':
			case 'w':
				my_vel.linear.x = lv;
				my_vel.angular.z = av*0;
				break;
			
			case 'S':
			case 's':
				my_vel.linear.x = -lv;
				my_vel.angular.z = av*0;
				break;
			
			case 'E':
			case 'e':
				my_vel.linear.x = lv;
				my_vel.angular.z = -av;
				break;
			
			case 'Q':
			case 'q':
				my_vel.linear.x = lv;
				my_vel.angular.z = av;
				break;
			case '1':
				my_vel.linear.x *= 1.2;
				my_vel.angular.z *= 1.2;
				break;
			case '2':
				my_vel.linear.x *= 0.8;
				my_vel.angular.z *= 0.8;
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

int main(int argc, char **argv){

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


