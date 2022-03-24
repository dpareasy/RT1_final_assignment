# final_assignment
================================

Parisi Davide Leo 4329668 

This assignment requires the development of a software architecture, in c++ language, using ROS operating system to drive a robot around a particular environment. The software relies on move_base and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request and let the robot execute one of the following behaviours:

* Autonomously reach a x,y coordinate inserted by the user

* Let the user drive the robot with the keyboard

* Let the user drive the robot assisting them to avoid collisions

## Content of the package ##

* world It's a folder that defines the world  within the robot shold move
* launch: it's a folder containing all the files with .launch extension needed to launch the simulation
* src: it's a folder containing all C++ executables used in this package
* CMakeList.txt: it's a file with informations about the compilation
* package.xml: it's a file with insofrmations about the compilation
* urdf: it's a folder containing the information about the robot
* config: it's a folder containing a file useful for simulation tools

In particulare the folder named 'src' contains:
- UI.cpp:  which is the node for the user interface 
- ReachTarget.cpp: which is the node for the autonomous navigation of the robot
- KeyboardnNavigation.cpp: which is the node for the navigation with the keyboard
- AssistedNavigation.cpp: which is the node for the navigation with assisted drive

## How to run ##

In order to launch the simulation there is a .lunch file that can be launched with the following command into the shell:

```
roslaunch final_assignment my_launch.launch
```

It includes the launch file for Gazebo and Rviz, the one for move_base algorithm and UI.cpp node, which according to the user choiche will launche the other three nodes with a syscall.

##Robot behaviours##

s

