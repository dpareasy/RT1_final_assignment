# final_assignment
================================

Parisi Davide Leo 4329668 

This assignment requires the development of a software architecture, in c++ language, using ROS operating system to drive a robot around a particular environment. The software relies on move_base and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request and let the robot execute one of the following behaviors:

* Autonomously reach a x, y coordinate inserted by the user;

* Let the user drive the robot with the keyboard;

* Let the user drive the robot assisting them to avoid collisions;

## Content of the package ##

* **world**: it's a folder that defines the world  within the robot shold move;
* **launch**: it's a folder containing all the files with .launch extension needed to launch the simulation;
* **src**: it's a folder containing all C++ executables used in this package;
* **CMakeList.txt**: it's a file with informations about the compilation;
* **package.xml**: it's a file with insoformations about the compilation;
* **urdf**: it's a folder containing the information about the robot;
* **config**: it's a folder containing a file useful for simulation tools;

In particular the folder named 'src' contains:
- `UI.cpp`:  which is the node for the user interface; 
- `ReachTarget.cpp`: which is the node for the autonomous navigation of the robot;
- `AssistedDrive.cpp`: which is the node for the navigation with assisted drive;

The one used for the simple navigation with the keyboard is the teleop_twist_keyboard node.

## How to run ##

In order to launch the simulation there is a .lunch file that can be launched with the following command into the shell:

```
roslaunch final_assignment my_launch.launch
```

It includes the launchfile for the simulation and the move_base package and for the UI.cpp node. 

```
<?xml version="1.0"?>
		
 <launch>
 
 	<!-- including all the launchfiles for the simulation-->
 	<include file = "$(find final_assignment)/launch/simulation_gmapping.launch" />
	<include file = "$(find final_assignment)/launch/move_base.launch" />
	
	<!--my UI node-->	
	<node name = "user_interface" pkg = "final_assignment" type = "user_interface_node" output = "screen" launch-prefix="xterm -e"  required = "true" />
</launch>

```
The UI.cpp node will run separately the other three nodes by using system calls:

- To run a single node:

```
system("rosrun final_assignment reach_target_node");

system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py");

```

- To run multiple nodes:

```
system("roslaunch final_assignment AssistedLaunch.launch");

```
## Robot behaviors ##

When the user launches the simulation, the robot, provided with a laser-scan, is spawned in a pre-built environment, waiting for a command from the user. Once a command is given, three different behaviors can be chosen:

```
1) The user can set a target within the environment in the ReachTarget node, that the robot must reach. A certain goal can also be canceled.  

2) The user can make the robot move around the environment, driven with the teleop_twist_keyboard node of ROS.

3) The user can make the robot move around the environment, driven with the teleop_twist_keyboard node of ROS, by avoiding the walls thanks to a system of assisted navigation implemented in the AssistedNavigation node.  If the robot gets too close to the walls, the laser scan with which the robot is equipped will detect it, and the system of the assisted drive will adjust the trajectory.
```

## About Software Architecture ##
I decided to divide the project into four different nodes to maintain a certain degree of modularity:

* The UI.cpp;
* The ReachTarget.cpp; 
* The AssistedDrive.cpp;
* The teleop_twist_keyboard.py;

From the UI.cpp, the user can choose the modality for moving the robot by typing commands from the keyboards. The user can decide on the autonomous navigation, the simple navigation with the keyboard, or the navigation with the keyboard provided by a system of the assisted drive. If the user chooses the first modality, the `UI.cpp` will launch the `ReachTarget.cpp` node. This node will publish on the `/move_base/goal` topic the point decided by the user that the robot has to reach. In the same node, another publisher has the function of canceling the goal, giving the user the faculty of changing the goal by publishing on the `/move_base/cancel` topic. Here the position of the robot and of the goal by subscribing respectively on `/move_base/feedback` topic and `/move_base/goal` topic. 
If the choice falls into the simple keyboard navigation, the `UI.cpp` will launch the `teleop_twist_keyboard.py` provided by ROS, and the user can drive the robot with specific commands by publishing velocities on `/cmd_vel` topic.
Finally, if the user chooses the third modality, the `UI.cpp` node will launch a launch file containing the `teleop_twist_keyboard.py` and the `AssistedDrive.cpp` node. The latter will get info on the distance from walls by subscribing from the `/scan` topic, on the current velocity of the robot on the map by subscribing from the `/cmd_vel_assisted` topic, and will publish the velocity values on the `/cmd_vel` topic.

## Pseudocodes ##

### UI.cpp ###

Choose modality:

```
for(;;){

	print the menu
	switch(modality){
	
		case 't':
			run reach_target_node;
			break;
		case 'k':
			run teleop_twist_keyboard node;
			break;
		case 'a':
			run AssistedDrive and teleop_twist_keyboard nodes; 
			break;
		case 'q':
			exit the simulation;
			break
		default:
			error message;
			break;
	
	}
}

```

### ReachTarget.cpp ###


Choose a command:

```
for(;;){

	print the menu
	switch(command){
		
		case 's':
			if(goalStatus){
				Set a gol for the robot;
			}
			else{
				inform the user of thr existence of a goal;
			}
			break;
		case 'c':
			Cancel the goal;
			break;
		case 'q':
			Exit the node and come back to the UI node;
			break;
		default:
			error message;
			break;
	
	}
}

```

#### Important Fucntions ####

SetGoal() function:

Used to set a goal to reach.

```
void SetGoal(){

	get x and y input;
	
	set inputs as x and y coord of the goal;
	
	publish the coordinates to /move_base/goal topic;
	
	goalStatus = true;
}

```



CancelGoal() function:

Used to cancel, if it exists, the last goal saved.

```
CancelGoal(){

	if (goalStatus){
	
		set the goal to cancel equal to the current goal;
		
		publish the id of the goal to cancel on the /move_base/cancel;
		
		goalStatus = false;
	}
	else{
	
		inform the user that no goal is set;
	}
}

```



CurrentGoal() function:

Used to store the position of the current goal.

It subscribes to the `/move_base/goal` topic.

```
CurrentGoal(){

	store the goal pose;
}

```




GoalStatus() function:

Used to check the status of the goal to inform the user that a goal is reached.

It subscribes to the `/move_base/feedback` topic the position of the robot within the environment.

```
GoalStatus(){

	set the goalID variable with the value of the actual goal id;
	
	if (goalStatus){
	
		if(robot_pose - goal_pose < error){
		
			inform the user that the goal is reached;
			
			cancel the goal;
			
			goalStatus = false;
		}
	}
}
```

### AssistedDrive.cpp ###

In this node one subscriber subscribes on `/cmd_vel_assisted` topic to get the robot velocities from the teleop_twist_keyboard node and a second one subscribes on `/scan` topic to get the distance of the robot from the walls. There is also a publisher to publish velocities that the robot has to move within the environment, publishing on `/cmd_vel`.

```
Divide ranges array into five subarrays;
//only three are used (front_right and front_left are not used)

array1 = right side;
array3 = front;
array5 = left side;

compute minimum value for each of the three arrays;

get velocities from the teleop_twist node and update current velocities;

if (front <= threashold){

	warn the user and ask to turn left or right;
	
	linear velocity = 0;
}

else if (left <= threshold){

	warn the user;
	
	turn right to adjust trajectory;
}

else if (right <= threshold){

	warn the user;
	
	turn left to adjust trajectory;
}

publish velocities;
```
## Limitations and possible improvements ##

One limitation of this program is that the robot does not save the values of the goals in a queue. To set another goal, the user has to wait until the robot reaches the first goal.
Another limitation of the program is that I haven't implemented a system that automatically cancels the unreachable points on the map. In this case, the robot will infinitely try to reach these points unless the user manually cancels them once noticing the unreachability with the graphical help of the map. 

It comes clear that two possible improvements to the code could be:
* The implementation of a system for saving in a queue the goals that the user could give in input to the program
* A way to state if the user has set an unreachable point for automatically deleting it.
