/**
*
*\file UI.cpp
*\brief Graphic user interface for the simulation
*\author Parisi Davide Leo S4329668
*\version 1.0
*\date 08/04/2022
*\details 
*Description:
*This node does not Publish or Subscribe on any topic. It's a graphical user interface that asks a user, with a menù, to select a modality for driving a robot around a particular 
*environment. The user's choice is saved in a global variable and handled by the program, which will run the node that manages the chosen modality.
**/
#include <stdlib.h>
#include "ros/ros.h"
#include <stdlib.h>

using namespace std;

char choice; ///< User's choice 


/**
*\brief Description of Menu() function:
*
*The aim of this function is to ask the user to choose
*from three different modalities or to close the program,
*and save the choice in a variable 
**/
char Menu(){
	system("clear");
	cout<<"::::::::::::::::::::::::::::|Menù|:::::::::::::::::::::::::::\r"<<endl;
	cout<<" Type T to set a goal position\r"<<endl;
	cout<<" Type K for driving the robot using the keyboard\r"<<endl;
	cout<<" Type A to equip the robot with a collision avoidance system\r"<<endl;
	cout<<" Type Q to exit the simulation\r"<<endl;
	cout<<":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::\r"<<endl;
	cin >> choice;
	return choice;
}


/**
*\brief Description of ChoiceHandling() function
*
*The aim of this function is to manage the user's 
*choice and run the nodes that perform the desired behavior 
**/
void ChoiceHandling(){
	
	//for loop for constantly asking the user to insert a command 
    for(;;){

        switch(Menu()){
        
            case 'T':
            case 't':
            	system("clear");
            	cout<<"PREPARING FOR AUTONOMOUS NAVIGATION"<<endl;
            	system("rosrun final_assignment reach_target_node");
            	
                break;
            case 'K':
            case 'k':
            	system("clear");
            	cout<<"PREPARING FOR KEYBOARD NAVIGATION \n\nWAIT FEW SWCONDS\n\n"<<endl;
            	cout <<"TYPE CTRL + C TO EXIT THE NODE"<<endl;
            	system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");
                
                break;
            case 'A':
            case 'a':
            	system("clear");
            	cout<<"PREPARING FOR KEYBOARD NAVIGATION WITH ASSISTED DRIVE \n\nWAIT FEW SWCONDS"<<endl;
            	system("roslaunch final_assignment AssistedLaunch.launch");
 
                break;
			case 'Q':
			case 'q':
				system("clear");
				cout<<"PROGRAM EXITING..."<<endl;
				exit (0);
            default:
            	system("clear");
                cout<<"\nINVALID INPUT, RETRY\n"<<endl;
                break;
        }
    }
}


int main(int argc, char **argv){
    //initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "user_interface_node");
	ros::NodeHandle nh;
	
	ChoiceHandling();
    
    return 0;
}


