#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Precise Action Sequence Controller with Rotation

This script executes a pre-defined sequence of movements, including a
180-degree yaw turn.

Sequence:
1. Move RIGHT by 1 meter.
2. Move FORWARD by 3 meters.
3. Rotate 180 degrees (YAW).
4. Descend to a final altitude of 0.8 meters.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
import numpy as np
import math
from tf.transformations import euler_from_quaternion

class SequenceControllerNode:
    def __init__(self):
        rospy.init_node("sequence_controller_node", anonymous=True)

        # --- 任务参数 ---
        self.MOVE_RIGHT_DISTANCE = 0.95
        self.MOVE_FORWARD_DISTANCE = 3.0
        self.FINAL_ALTITUDE = 1

        # --- 控制参数 ---
        self.CONTROL_FREQUENCY = 20.0
        self.LINEAR_VELOCITY = 0.4
        self.ALTITUDE_VELOCITY = 0.2
        self.ANGULAR_VELOCITY = math.radians(45.0) # 旋转速度 (45度/秒)
        self.POSITION_TOLERANCE = 0.1
        self.YAW_TOLERANCE = math.radians(5.0)     # 5度的容忍误差

        # --- 状态机 (新增 ROTATE_180) ---
        self.states = ['MOVE_RIGHT', 'MOVE_FORWARD', 'ROTATE_180', 'DESCEND', 'FINISHED']
        self.current_state = self.states[0]

        # --- 状态变量 ---
        self.current_pose = None
        self.pose_received = False
        self.target_position = None
        self.target_yaw = None # 用于存储旋转目标

        # --- ROS ---
        rospy.Subscriber("/mavros/odometry/out", Odometry, self.odometry_cb)
        self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.control_rate = rospy.Rate(self.CONTROL_FREQUENCY)
        
    def odometry_cb(self, msg):
        self.current_pose = msg.pose.pose
        if not self.pose_received:
            self.pose_received = True
            
    def normalize_angle(self, angle):
        """将角度标准化到 -pi 到 pi 之间"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def run(self):
        rospy.loginfo("Waiting for first odometry message...")
        while not rospy.is_shutdown() and not self.pose_received:
            self.control_rate.sleep()
        
        rospy.loginfo("System ready. Starting action sequence.")

        while not rospy.is_shutdown() and self.current_state != 'FINISHED':
            current_position = np.array([self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z])
            q = self.current_pose.orientation
            _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            cmd_vel = Twist()

            # --- 状态机逻辑 ---
            if self.current_state == 'MOVE_RIGHT' or self.current_state == 'MOVE_FORWARD':
                if self.target_position is None:
                    # 进入新状态，计算平移目标点
                    if self.current_state == 'MOVE_RIGHT':
                        rospy.loginfo("State: MOVE_RIGHT. Calculating target...")
                        right_vec_x = math.cos(current_yaw - math.pi / 2) * self.MOVE_RIGHT_DISTANCE
                        right_vec_y = math.sin(current_yaw - math.pi / 2) * self.MOVE_RIGHT_DISTANCE
                        self.target_position = current_position + np.array([right_vec_x, right_vec_y, 0])
                    elif self.current_state == 'MOVE_FORWARD':
                        rospy.loginfo("State: MOVE_FORWARD. Calculating target...")
                        forward_vec_x = math.cos(current_yaw) * self.MOVE_FORWARD_DISTANCE
                        forward_vec_y = math.sin(current_yaw) * self.MOVE_FORWARD_DISTANCE
                        self.target_position = current_position + np.array([forward_vec_x, forward_vec_y, 0])
                
                # --- 平移控制 ---
                distance_to_target = np.linalg.norm(self.target_position[:2] - current_position[:2])
                if distance_to_target > self.POSITION_TOLERANCE:
                    direction_vector = (self.target_position - current_position) / np.linalg.norm(self.target_position - current_position)
                    cmd_vel.linear.x = direction_vector[0] * self.LINEAR_VELOCITY
                    cmd_vel.linear.y = direction_vector[1] * self.LINEAR_VELOCITY
                    z_error = current_position[2] - self.current_pose.position.z # 维持高度
                    cmd_vel.linear.z = np.clip(z_error * 0.5, -0.2, 0.2)
                else:
                    self.go_to_next_state() # 到达目标，切换状态

            elif self.current_state == 'ROTATE_180':
                if self.target_yaw is None:
                    # 进入旋转状态，计算目标角度
                    rospy.loginfo("State: ROTATE_180. Calculating target yaw...")
                    self.target_yaw = self.normalize_angle(current_yaw + 0)
                    rospy.loginfo(f"Current Yaw: {math.degrees(current_yaw):.2f}, Target Yaw: {math.degrees(self.target_yaw):.2f}")

                # --- 旋转控制 ---
                yaw_error = self.normalize_angle(self.target_yaw - current_yaw)
                if abs(yaw_error) > self.YAW_TOLERANCE:
                    cmd_vel.angular.z = np.sign(yaw_error) * self.ANGULAR_VELOCITY
                else:
                    self.go_to_next_state() # 到达目标，切换状态
            
            elif self.current_state == 'DESCEND':
                if self.target_position is None:
                    rospy.loginfo("State: DESCEND. Setting final altitude...")
                    self.target_position = np.array([current_position[0], current_position[1], self.FINAL_ALTITUDE])

                # --- 下降控制 ---
                z_error = self.target_position[2] - current_position[2]
                if abs(z_error) > self.POSITION_TOLERANCE:
                    cmd_vel.linear.z = np.sign(z_error) * self.ALTITUDE_VELOCITY
                else:
                    self.go_to_next_state() # 到达目标，切换状态

            self.vel_pub.publish(cmd_vel)
            self.control_rate.sleep()

        rospy.loginfo("Action sequence completed.")
        stop_cmd = Twist()
        while not rospy.is_shutdown():
            self.vel_pub.publish(stop_cmd)
            self.control_rate.sleep()

    def go_to_next_state(self):
        """切换到下一个状态并重置目标"""
        rospy.loginfo(f"State '{self.current_state}' finished.")
        current_state_index = self.states.index(self.current_state)
        self.current_state = self.states[current_state_index + 1]
        self.target_position = None
        self.target_yaw = None
        # 短暂悬停
        for _ in range(10):
            self.vel_pub.publish(Twist())
            self.control_rate.sleep()

if __name__ == "__main__":
    try:
        controller = SequenceControllerNode()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")