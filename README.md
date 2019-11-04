# First Publisher/Subscriber
Tutorials for ROS
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

##Overview
This is a beginners tutorial for Publisher Subscriber as given by ROS wiki.
Publisher - src/talker.cpp
Subscriber - src/listener.cpp

##Dependency
This module is built using ROS Kinetic and catkin.

ROS Kinetic installation steps - http://wiki.ros.org/kinetic/Installation/Ubuntu

##Clone and Build
Clone package into catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/arunabaijal/beginner_tutorials.git
cd ~/catkin_src
source /opt/ros/kinetic/setup.bash
catkin_make
catkin_make install
source ~/catkin_ws/devel/setup.bash

##Run
Open a new terminal and run the following commands for master
roscore

In another new terminal run the follwoing commands for publisher
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash 
rosrun beginner_tutorials talker

In a third new terminal run the following commands for subscriber
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash 
rosrun beginner_tutorials listener

## Run using launch file
After building the package
Open a new terminal and run the following commands for master
roscore

Open a new terminal and run following command for both publisher and subscriber
roslaunch beginner_tutorials talkerlistener.launch frequency:=1

To run with default frequency of 10 use command
roslaunch beginner_tutorials talkerlistener.launch

## Changing message using service
To change message being published open a new terminal and run following command while publisher is running
rosservice call /ModifyMessage "Change message!"
