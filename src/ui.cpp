#include <stdlib.h>
#include "ros/ros.h"
//#include "final_assignment/Target.h"
//#include "final_assignment/Keyboard.h"
//#include "final_assignment/Avoidcoll.h"

using namespace std;
char choice;


char Menu()
{
	cout<<"::::::::::::::::::::::::::::::Menù::::::::::::::::::::::::::::::"<<endl;
	cout<<"*Type t/T to set a position targe"<<endl;
	cout<<"*Type k/K for driving the robot using the keyboard"<<endl;
	cout<<"*Type a/A to equip the robot with a collision avoidance system "<<endl;
	cout<<"*Type q/Q to exit the simulation"<<endl;
	cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<endl;
	cin >> choice;
	return choice;
}

int main(int argc, char **argv)
{
    //initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "user_interface_node");
	ros::NodeHandle nh;
	
    //while loop for constantly asking the user to insert a command 
    while(1)
    {

        switch(Menu())
        {
            case 'T':
            case 't':
            	cout<<"Reaching target modality has been chosen"<<endl;
            	system("rosrun final_assignment reach_target_node");
            	
                break;
            case 'K':
            case 'k':
            	cout<<"Keyboard navigation modality has been chosen"<<endl;
            	system("rosrun final_assignment teleopkey_node");
                
                break;
            case 'A':
            case 'a':
            	cout<<"Assistive drive navigation modality has been chosen"<<endl;
            	system("rosrun final_assignment teleopkey_avoidcoll_node");
                
                break;
			case 'Q':
			case 'q':
				cout<<"Program exiting..."<<endl;
				return 0;
            default:
                cout<<"\nInvalid input, retry\n"<<endl;
                break;
        }
    }
    return 0;
}


