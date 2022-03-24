#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "geometry_msgs/PointStamped.h"
#include "time.h"
#include <iostream> 
#include <sstream> 
#include <regex>

using namespace std;

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Reach_point function
//publish the target point
ros::Publisher pubGoal; 
ros::Publisher pubCancel;
//initializing the goal variable
move_base_msgs::MoveBaseActionGoal my_goal;

//X and Y coord
double X, Y;
//set a boolean to state if the goal is set 
bool goalState = false;

//Here follows two function to save the x and y coordinates
//values. The functions check if the inputs are number or not
//if they are, return the values
double SetX()
{	
	double x;
   	 while (true){
        	cout << "Enter the value for the X coordinate" << endl;
        	cin.clear();
        	while(cin.get() != '\n');
        	{	
        		cin >> x;
        	}
        	if (!cin.fail()){
        	cout<<"x = "<<x<<endl;
           	return x;
        	}
    	}
}

double SetY()
{
	double y;
   	 while (true){
        	cout << "Enter the value for the Y coordinate" << endl;
        	cin.clear();
        	while(cin.get() != '\n');
        	{	
        		cin >> y;
        	}
        	if (!cin.fail()){
        	cout<<"y = "<<y<<endl;
           	return y;
        	}
    	}
}


void Menu()
{	
		
	cout<<"____________________Menù____________________"<<endl;
	cout<<"*Type 's' if you wanto to set a goal point"<<endl;
	cout<<"*Type 'c' if you want to cancel the last goal"<<endl;
	cout<<"*Type 'q' if you want to exit the node"<<endl;
	cout<<"____________________________________________"<<endl;
}

void InputCoord()
{
	X = SetX();
	Y = SetY();
}

void ReachGoal()
{
	my_goal.goal.target_pose.pose.position.x = X;
	my_goal.goal.target_pose.pose.position.y = Y;
	// set the frame_id
	my_goal.goal.target_pose.header.frame_id = "map";
	//set the quaternion module equal to 1
	my_goal.goal.target_pose.pose.orientation.w = 1;
	pubGoal.publish(my_goal);
}

void CancelGoal()
{
	actionlib_msgs::GoalID first_goal;
	goalState = false;
	pubCancel.publish(first_goal);
}

void Decision()
{
	for(;;)
	{
		//system("clear");
		Menu();
		char decision;
		cin >> decision;
		switch (decision)
		{
			case 'S':
			case 's':
				goalState = true;
				InputCoord();
				ReachGoal();
				system("clear");
				break;
			case 'C':
			case 'c':
				if (goalState)
				{
					cout<<"cancelling the goal"<<endl;
					CancelGoal();
				}
				else
				{
					cout<<"No goal set"<<endl;
				}
				sleep(1);
				//system("clear");
				break;
			case 'Q':
			case 'q':
				cout<<"Exiting the node..."<<endl;
				CancelGoal();
				system("clear");
				exit(0);
				break;
			default:
				cout<<"Invalid input, rety!"<<endl;
				break;			
		}
	}
}

void GoalFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
	ROS_INFO("PoseSubscriber@[%f,%f,%f]", msg -> feedback.base_position.pose.position.x, msg -> feedback.base_position.pose.position.y, msg -> feedback.base_position.pose.position.z);
}

int main(int argc, char **argv)
{
	
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "reach_target_node");
	ros::NodeHandle nh;
	
	ros::Subscriber subPose = nh.subscribe("/move_base/feedback", 1, GoalFeedback);
	
	//define the publisher to send message for the velocity of the robot
	pubGoal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	
	pubCancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
	
	Decision();
	
	ros::spin();	
	return 0;
}


