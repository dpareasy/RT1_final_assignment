# final_assignment
================================

Parisi Davide Leo 4329668 

This assignment requires the development of a software architecture, in c++ language, using ROS operating system to drive a robot around a particular environment. The software relies on move_base and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request and let the robot execute one of the following behaviours:

* Autonomously reach a x,y coordinate inserted by the user

* Let the user drive the robot with the keyboard

* Let the user drive the robot assisting them to avoid collisions

## Content of the package ##

* **world**: it's a folder that defines the world  within the robot shold move
* **launch**: it's a folder containing all the files with .launch extension needed to launch the simulation
* **src**: it's a folder containing all C++ executables used in this package
* **CMakeList.txt**: it's a file with informations about the compilation
* **package.xml**: it's a file with insofrmations about the compilation
* **urdf**: it's a folder containing the information about the robot
* **config**: it's a folder containing a file useful for simulation tools

In particulare the folder named 'src' contains:
- `UI.cpp`:  which is the node for the user interface 
- `ReachTarget.cpp`: which is the node for the autonomous navigation of the robot
- `AssistedNavigation.cpp`: which is the node for the navigation with assisted drive

The one used for the simple navigation with the keyboard is the teleop_twist_keyboard node.

## How to run ##

In order to launch the simulation there is a .lunch file that can be launched with the following command into the shell:

```
roslaunch final_assignment my_launch.launch
```

It includes the launchfile for the simulation, the move_base package and the UI.cpp node which will run separately the other three nodes with a system call.

## Robot behaviors ##

When the user launches the simulation, the robot is spawned in a pre-built environment, waiting for a command from the user. Once a command is given, the user can choose three different behaviors:

```
1 - The user will set a target on the environment in the ReachTarget node, and the robot must reach it.  

2 - The Robot can move around the environment, driven by the user thanks to the teleop_twist_keyboard node of ROS.

3 - The robot can move around the environment, driven by the user thanks to the teleop_twist_keyboard node of ROS, by avoiding the walls thanks to a system of assisted navigation implemented in the AssistedNavigation node.
```

Concerning the ReachTarget node, I have implemented a timeout to avoid the robot trying to reach a point out of the map. If the target point won't be reached by the robot after a certain time, the goal will be canceled.

## About Software Architectures ##
I decided to divide the project into four different nodes: 
* The UI.cpp;
* The ReachTarget.cpp; 
* The AssistedDrive.cpp;
* The teleop_twist_keyboard.py;
From the UI.CPP, the user can choose the modality for moving the robot by typing commands from the keyboards. The user can decide on the autonomous navigation, the simple navigation with the keyboard, or the navigation with the keyboard assisted by a system of the assisted drive. If the user chooses the first modality, the UI.CPP will launch the ReachTarget node. This node will publish on the `/move_base/goal` topic the point decided by the user that the robot has to reach. In the same node, another publisher has the function of canceling the goal, giving the user the faculty of changing the robot goal by publishing on the `/move_base/cancel` topic.
If the choice falls into the simple keyboard navigation, the UI.cpp will launch the teleop_twist_keyboard.py provided by ROS, and the user can drive the robot with specific commands.
Finally, if the user chooses the third modality, the UI.cpp node will launch a launch file containing the teleop_twist_keyboard.py and the AssistedDrive.cpp node. The latter will get info on the distance from walls by subscribing from the `/scan` topic, on the current velocity of the robot on the map by subscribing from the `/cmd_vel_assisted` topic, and will publish the velocity values on the `/cmd_vel` topic. If the robot gets too close to the walls, the laser scan with which the robot is equipped will detect it, and the system of the assisted drive will adjust the trajectory.




