# 🚁 Real-World UAV Control & Mission Package

## Overview
This directory contains the core ROS packages and scripts designed for **real-world UAV deployment**. Unlike the simulation modules, these scripts are tuned for physical flight dynamics, handling onboard sensor data (camera/LiDAR) and communicating with the flight controller (FCU) via MAVROS.

The core package, **`drone_ctrl_pkg`**, integrates autonomous navigation, visual perception, and manual intervention capabilities.

## 📂 Key Features

### 1. Coordinate-Based Navigation
* **Functionality**: Allows the UAV to fly to specific local coordinates $(x, y, z)$ in the `OFFBOARD` mode.
* **Logic**: Sends setpoints to MAVROS topics (`/mavros/setpoint_position/local`) to execute precise movement trajectories in the real environment.

### 2. Visual Perception (Balloon Detection)
* **Target**: Autonomous detection and engagement of red balloons.
* **Algorithm**: Utilizes the onboard camera to process images using OpenCV. It identifies red color regions, calculates the centroid, and generates control commands to approach the target.

### 3. Keyboard Teleoperation
* **Functionality**: A safety and testing tool that allows the operator to send coordinate setpoints or velocity commands using the keyboard.
* **Use Case**: Essential for field testing, allowing for manual adjustments of the drone's position without switching out of Offboard mode.

## 🛠️ Hardware Requirements
* **Airframe**: Quadrotor equipped with a PX4-based Flight Controller (Pixhawk/Durandal).
* **Onboard Computer**: Jetson Nano / Raspberry Pi / Manifold (Running ROS Noetic/Melodic).
* **Sensors**:
    * Down-facing or Front-facing Camera (USB/CSI).
    * LiDAR (Optional, for altitude/position hold).
* **Communication**: MAVROS bridge establishing a link between the Onboard Computer and FCU.

## 🚀 Usage Guide

### 1. Prerequisites
Ensure the physical drone is powered on, GPS lock is acquired (if used), and the EKF is stable.
```bash
# Verify MAVROS connection
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
rostopic echo /mavros/state
2. Launching the Control Package
To start the main control node (coordinate navigation):

Bash

rosrun drone_ctrl_pkg <your_coordinate_script>.py
3. Balloon Detection Mission
To execute the visual servoing task:

Bash

rosrun drone_ctrl_pkg balloon_detect.py
4. Keyboard Control
To enable manual coordinate input:

Bash

rosrun drone_ctrl_pkg keyboard_ctrl.py
⚠️ Safety Notes
Always have a safety pilot with an RC transmitter ready to switch to STABILIZED or LAND mode immediately in case of code failure.

Ensure the operational area is clear of obstacles before executing coordinate-based autonomous flights.
