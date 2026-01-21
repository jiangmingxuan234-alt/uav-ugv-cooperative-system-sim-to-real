#!/bin/bash


cd /root/catkin_ws/src/sensor_pkg && python3 main.py &
使用 sleep 命令等待1.1秒
sleep 20

# 在新终端窗口中运行第三个命令
xterm -hold -e "roslaunch faster_lio rflysim.launch" &

