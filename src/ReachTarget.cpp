/**
*\file ReachTarget.cpp
*\brief Autonomous navigation modality
*\author Parisi Davide Leo S4329668
*\version 1.0
*\date 08/04/2022
*\details
*Subscribes to: <BR>
* °/move_base/feedback
* °/move_base/goal
*Publishes to: <BR>
* °/move_base/goal
* °/move_base/cancel
*
*Description:
*
*This node simulate the autonomous nvigation of a robt within the environment. It asks the user to insert a goal point.
*Once inserted the user can decide to cancel the actual goal or to exit the modality. If a goal point is already defined
*the user can't choose another unless he cancel it.
**/

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
ros::Publisher pubGoal; ///< Publisher for publishing the goal point

ros::Publisher pubCancel; ///< Publisher to cancel the goal

move_base_msgs::MoveBaseActionGoal my_goal; ///< Variable for saving the chosen goal

actionlib_msgs::GoalID lastGoal; ///< Variable for the last goal to cancel 

std::string goalID; ///< Variable used to save the actual goal


double X, Y; ///< X and Y coordinates of the goal


float xG; ///< X variable to store the goal
float yG; ///< Y variable to store the goal

 
bool goalStatus = false; ///< Boolean to state if a goal is set


float error = 0.5; ///< Error for the position of the robot wrt the goal point


/**
*\brief Description of SetX() function:
*
*The aim of this function is to check if the inputs are numbers or not
*if they are, return the values, in the other case the function
*will continue to ask to insert a value for the X coordinate.
**/
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


/**
*\brief Description of SetY() function:
*
*The aim of this function is to check if the inputs are numbers or not
*if they are, return the values, in the other case the function
*will continue to ask to insert a value for the Y coordinate.
**/
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


/**
*\brief Description of Menu() function:
*
*The aim of this function is to ask the user to give commands to
*the robot. The user can choose for publishin a goal, canceling the
*last goal or exiting the autonomous navigation modality.
**/
void Menu(){	

	cout<<"____________________Menù____________________\r"<<endl;
	cout<<"*Type 's' if you wanto to set a goal point\r"<<endl;
	cout<<"*Type 'c' if you want to cancel the last goal\r"<<endl;
	cout<<"*Type 'q' if you want to change modality\r"<<endl;
	cout<<"____________________________________________\r"<<endl;
}


/**
\brief Description of InputCoord() function:
*
*In this function the x and y coordinate gotten as an input 
*are saved in two global variables
**/
void InputCoord(){

	X = SetX();
	Y = SetY();
	system("clear");
}


/**
*\brief Description of SetGoal() function:
*
*In this function the goal point is set and the 
*variable which state that the goal is saved, and the
*robot is moving to it. 
**/
void SetGoal(){

	InputCoord();
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


/**
*\brief Description of CancelGoal() function:
*
*In this function the current goal saved is canceled 
*and the variable that gives information about the 
*goal status is set to false, to state that no goal is set.
*If no goal is set, nothing would happen. 
**/
void CancelGoal(){

	if(goalStatus){
		
		system("clear");
		//set the goal to cancel equal to the current goal
		lastGoal.id = goalID;
		
		cout<<"Canceling goal"<<endl;
		
		pubCancel.publish(lastGoal);
		
		//set the status equal to false
		goalStatus = false;
	}
	else
		cout<<"No goal set"<<endl;
}


/**
*\brief Description of Decision() function:
*
*In this function the input from the user is saved and andled to perform the 
*desired behaviour.
**/
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
					SetGoal();
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


/**
*\brief Description of GoalStatus() function:
*
*In this function the goal status is saved. When the robot reaches the target
*a message will notifies the user that the goal is reached, theen the robot is 
*ready for another target.
**/
void GoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){

	// set the goalID variable with the value of the actual goal id
	goalID = msg -> status.goal_id.id;
	
	// check the presence of a goal
	if(goalStatus){
	
		// check the distance of both coordinates to see if the robot is near the goal
		if(abs(msg -> feedback.base_position.pose.position.x - xG) <= error && abs(msg -> feedback.base_position.pose.position.y - yG) <= error)
		{
			system("clear");
			
			// print
			std::cout << "Goal reached successfully!\n";
			
			// cancel goal
			CancelGoal();
			
			
			//showing the menu to the user
			Menu();
		}
		
	}
}


/**
*\brief Description of CurrentGoal() function:
*
*In this function the goal position is saved to inform the user
*about the next target of the robot.
**/
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
	pubCancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
	
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


