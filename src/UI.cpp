#include <stdlib.h>
#include "ros/ros.h"
#include <stdlib.h>

using namespace std;
char choice;

char Menu(){
	system("clear");
	cout<<"::::::::::::::::::::::::::::|Menù|:::::::::::::::::::::::::::"<<endl;
	cout<<" Type T to set a goal position"<<endl;
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
            	cout<<"PREPARING FOR AUTONOMOUS NAVIGATION"<<endl;
            	system("rosrun final_assignment reach_target_node");
            	
                break;
            case 'K':
            case 'k':
            	cout<<"PREPARING FOR KEYBOARD NAVIGATION \n\nWAIT FEW SWCONDS\n\n"<<endl;
            	system("Color B5");
            	cout <<"TYPE CTRL + C TO EXIT THE NODE"<<endl;
            	system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");
                
                break;
            case 'A':
            case 'a':
            	cout<<"PREPARING FOR KEYBOARD NAVIGATION WITH ASSISTED DRIVE \n\nWAIT FEW SWCONDS"<<endl;
            	system("roslaunch final_assignment AssistedLaunch.launch");
 
                break;
			case 'Q':
			case 'q':
				cout<<"PROGRAM EXITING..."<<endl;
				return 0;
            default:
                cout<<"\nINVALID INPUT, RETRY\n"<<endl;
                break;
        }
    }
    return 0;
}


