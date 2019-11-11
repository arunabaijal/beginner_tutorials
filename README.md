[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# First Publisher/Subscriber
Tutorials for ROS

## Overview
This is a beginners tutorial for Publisher Subscriber as given by ROS wiki.
Publisher - src/talker.cpp
Subscriber - src/listener.cpp

## Dependency
This module is built using ROS Kinetic and catkin.

ROS Kinetic installation steps - http://wiki.ros.org/kinetic/Installation/Ubuntu

## Clone and Build
Clone package into catkin workspace
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/arunabaijal/beginner_tutorials.git
  cd ~/catkin_ws
  source /opt/ros/kinetic/setup.bash
  catkin_make
  catkin_make install
  source ~/catkin_ws/devel/setup.bash
  ```

## Run
Open a new terminal and run the following commands for master
roscore

In another new terminal run the follwoing commands for publisher
  ```
  cd ~/catkin_ws/
  source ~/catkin_ws/devel/setup.bash 
  rosrun beginner_tutorials talker
  ```

In a third new terminal run the following commands for subscriber
  ```
  cd ~/catkin_ws/
  source ~/catkin_ws/devel/setup.bash 
  rosrun beginner_tutorials listener
  ```

## Run using launch file
After building the package
Open a new terminal and run the following commands for master
  ```
  roscore
  ```

Open a new terminal and run following command for both publisher and subscriber
  ```
  roslaunch beginner_tutorials talkerlistener.launch frequency:=1
  ```

To run with default frequency of 10 use command
  ```
  roslaunch beginner_tutorials talkerlistener.launch
  ```

## Changing message using service
To change message being published open a new terminal and run following command while publisher is running
  ```
  rosservice call /ModifyMessage "Change message!"
  ```

# Inspecting TF Frames
The talker.cpp publishes /tf topic of a non zero static tf frames called /talk with respect to the /world frame. 
 
 ```
 cd ~/catkin_ws
 rosrun rqt_tf_tree rqt_tf_tree
 ```
To echo the values type the following in a new terminal: 
 ```
 cd ~/catkin_ws
 rosrun tf tf_echo /world /talk
 ```
 
View_frames will produce a diagram of the broadcaster frame. While running the demo type the following in the new terminal
 ```
 cd ~/catkin_ws
 rosrun tf view_frames
 ```
Above command will produce a pdf file which can be viewed in the catkin workspace. An example of this pdf can be viewed in the results folder
 
# Running ROS Unit Tests
To run the ros unit testing type the following in a new terminal 

## Using Launch File 
```
cd ~/catkin_ws
rostest beginner_tutorials test.launch
```
## Using catkin_make
```
cd ~/catkin_ws
catkin_make run_tests_beginner_tutorials
```
sample output
```
  ... logging to /home/aruna/.ros/log/rostest-aruna-VivoBook-15-ASUS-Laptop-X570UD-8136.log
  [ROSUNIT] Outputting test results to /home/aruna/.ros/test_results/beginner_tutorials/rostest-test_test.xml
  [ERROR] [1573491493.343802475]: Rate has to be positive!
  [Testcase: testtalkerTest] ... ok

  [ROSTEST]-----------------------------------------------------------------------

  [beginner_tutorials.rosunit-talkerTest/ModifyMessage][passed]

  SUMMARY
   * RESULT: SUCCESS
   * TESTS: 1
   * ERRORS: 0
   * FAILURES: 0

  rostest log file is in /home/aruna/.ros/log/rostest-aruna-VivoBook-15-ASUS-Laptop-X570UD-8136.log

```

# Playing bag files 
A recorded ros bag file is located in the results folder. To play the ros bag file type the following commands: 

In a new terminal 
```
roscore
```

Open another new terminal
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```

In an another new terminal
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play BagFile.bag
```
The /chatter messages that have been recorded can be viewed in the listner node 

# Recording bag files with launch file 
```
roslaunch beginner_tutorials week10HW.launch StartRec:=true
```


