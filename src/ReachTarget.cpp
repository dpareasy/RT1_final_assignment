#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include <string>
#include <chrono>


using namespace std;


//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Reach_point function
//publish the target point
ros::Publisher pubGoal; 
ros::Publisher pubCancel;

//initializing the goal variable
move_base_msgs::MoveBaseActionGoal my_goal;

//variable for the last goal to cancel
actionlib_msgs::GoalID lastGoal;


//variable used to save the actual goal
std::string goalID;

 //X and Y coord
double X, Y;


// define variables to store the goal
float xG;
float yG;

//set a boolean to state if the goal is set 
bool goalStatus = false;


//define an error threshold 
//used for the position of the robot
float error = 0.5;


//Here follows two function to save the x and y coordinates
//values. The functions check if the inputs are number or not
//if they are, return the values, in the other case the function
//will continue to ask to insert a value
//set X coordinate
double SetX(){
	
	double x;
   	 while (true){
        	cout << "Enter the value for the X coordinate" << endl;
        	cout << "X = " ;
        	cin.clear();
        	while(cin.get() != '\n');
        	{	
        		cin >> x;
        	}
        	if (!cin.fail()){
           	return x;
        	}
    	}
}


//set Y coordinate
double SetY(){

	double y;
   	 while (true){
        	cout << "Enter the value for the Y coordinate" << endl;
        	cout << "Y = " ;
        	cin.clear();
        	while(cin.get() != '\n');
        	{	
        		cin >> y;
        	}
        	if (!cin.fail()){
        	return y;
        	}
    	}
}


//Menù for the user
void Menu(){	

	cout<<"____________________Menù____________________\r"<<endl;
	cout<<"*Type 's' if you wanto to set a goal point\r"<<endl;
	cout<<"*Type 'c' if you want to cancel the last goal\r"<<endl;
	cout<<"*Type 'q' if you want to exit the node\r"<<endl;
	cout<<"____________________________________________\r"<<endl;
}


//function to save the input coordinates
void InputCoord(){

	X = SetX();
	Y = SetY();
	system("clear");
}


//function for setting the coordinates of the goal
void ReachGoal(){

	//setting x and y coordinates
	my_goal.goal.target_pose.pose.position.x = X;
	my_goal.goal.target_pose.pose.position.y = Y;
	// set the frame_id
	my_goal.goal.target_pose.header.frame_id = "map";
	//set the quaternion module equal to 1
	my_goal.goal.target_pose.pose.orientation.w = 1;
	
	//set the status equal to true
	goalStatus = true;
	
	pubGoal.publish(my_goal);
}


//function to cancel the goal
void CancelGoal(){

	if(goalStatus){
		
		cout<<"canceling the goal"<<endl;
		//set the goal to cancel equal to the current goal
		lastGoal.id = goalID;
		
		pubCancel.publish(lastGoal);
		
		//set teh status equal to false
		goalStatus = false;
	}
	else
		cout<<"No goal set"<<endl;
}


//user decision handler
void Decision(){

	for(;;){
	
		Menu();
		char decision;
		cin >> decision;
		switch (decision){
			case 'S':
			case 's':
				if (goalStatus){
					system("clear");
					cout<<"A goal is alreasy set, wait until the reachment of the target or type 'c' to cancel the goal and set another goal"<<endl;
					cout<<"Goal set, the robot is moving to ["<< xG <<", " << yG <<"]"<<endl;
				}
				else{
					
					system("clear");
					InputCoord();
					ReachGoal();
				}
				break;
			case 'C':
			case 'c':
				
				CancelGoal();
				sleep(1);
				system("clear");
				
				break;
			case 'Q':
			case 'q':
			
				cout<<"Exiting the node..."<<endl;
				CancelGoal();
				system("clear");
				exit(0);
				
				break;
			default:
			
				cout<<"Invalid input, retry!"<<endl;
				
				break;			
		}
	}
}


// function to take the status: in particular the actual goal id
void GoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){

	// set the goalID variable with the value of the actual goal id
	goalID = msg -> status.goal_id.id;
	
	// check the presence of a goal
	if(goalStatus){
	
		// check the distance of both coordinates to see if the robot is near the goal
		if(abs(msg -> feedback.base_position.pose.position.x - xG) <= error && abs(msg -> feedback.base_position.pose.position.y - yG) <= error)
		{
			system("clear");
			// cancel goal
			CancelGoal();

			system("clear");
			// print
			std::cout << "Goal reached successfully!\n";
			
			//showing the menu to the user
			Menu();
		}
		
	}
}


// function to store the current goal of the robot
void CurrentGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& m)
{
	// get x coordinate of the current goal
	xG = m -> goal.target_pose.pose.position.x;
	// get y coordinate of the current goal
	yG = m -> goal.target_pose.pose.position.y;
	
	cout<<"Goal set, the robot is moving to ["<< xG <<", " << yG <<"]"<<endl;
}


int main(int argc, char **argv){
	
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "reach_target_node");
	ros::NodeHandle nh;
	
	// publisher to send message for the goal of the robot
	pubGoal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	
	// publisher to send message for canceling the goal
	pubCancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
	
	// subscribe to the topic feedback to have the status always available and updated
	ros::Subscriber sub = nh.subscribe("/move_base/feedback", 1, GoalStatus);
	
	// subscribe to the topic goal to have the current status always available and updated
	ros::Subscriber subG = nh.subscribe("/move_base/goal", 1, CurrentGoal);
	
	//multi-threading
	ros::AsyncSpinner spinner(4);
	
	spinner.start();
	
		Decision();//enter in the menù	
	
	spinner.stop();
	
	
	return 0;
}


