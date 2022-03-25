#include <stdlib.h>
#include "ros/ros.h"

using namespace std;
char choice;

char Menu(){

	cout<<"::::::::::::::::::::::::::::|Menù|:::::::::::::::::::::::::::"<<endl;
	cout<<" Type T to set a position target"<<endl;
	cout<<" Type K for driving the robot using the keyboard"<<endl;
	cout<<" Type A to equip the robot with a collision avoidance system "<<endl;
	cout<<" Type Q to exit the simulation"<<endl;
	cout<<":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<endl;
	cin >> choice;
	return choice;
}

int main(int argc, char **argv){
    //initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "user_interface_node");
	ros::NodeHandle nh;
	
    //while loop for constantly asking the user to insert a command 
    for(;;){

        switch(Menu()){
        
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
            	system("rosrun final_assignment teleopkey_avoidcoll_node  &");
            	system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_assisted");
            	
 
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


