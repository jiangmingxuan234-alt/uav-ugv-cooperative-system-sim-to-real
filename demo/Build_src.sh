#!/bin/bash

mkdir -p ~/catkin_ws

unzip src.zip -d ~/catkin_ws

cd ~/catkin_ws/src

chmod 777 -R *

cd livox_ros_driver2/
./build.sh ROS1

cd ..

cd ..

catkin_make