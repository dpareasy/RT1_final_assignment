#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "final_assignment/Target.h"
#include "time.h"

using namespace std;


//declare coordinates X and Y
double X;
double Y;

//function to save the coordinates values
void SetXY(double &x, double &y)
{
   cout<<"Choose the x coordinate"<<endl;
   cout << "x = ";
   cin >> x;
   cout<<"Choose the y coordinate"<<endl;
   cout << "y = ";
   cin >> y;
  
}

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it also in the Reach_point function
//publish the target point
ros::Publisher pub; 

//initializing the goal variable
move_base_msgs::MoveBaseActionGoal my_goal;


void Reach_point()
{
	SetXY(X,Y);
	my_goal.goal.target_pose.pose.position.x = X;
	my_goal.goal.target_pose.pose.position.y = Y;
	// set the frame_id
	my_goal.goal.target_pose.header.frame_id = "map";
	//set the quaternion module equal to 1
	my_goal.goal.target_pose.pose.orientation.w = 1;
	pub.publish(my_goal);
}


int main(int argc, char **argv)
{
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "reach_target_node");
	ros::NodeHandle nh;
	//define the publisher to send message for the velocity of the robot
	pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	Reach_point();
	ros::spin();
	return 0;
}


