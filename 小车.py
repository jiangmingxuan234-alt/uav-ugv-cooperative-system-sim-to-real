#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无限循环直线轨迹跟踪（ROS 2）- 带横向误差修正
运行：
    ros2 run <pkg> straight_line_loop_corrected
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

# ----------- 参数 -----------
WAYPOINTS = [
    (0.0, 0.0),
    (-6.31 + 7.17, -0.72), # (0.86, -0.72)
    (-3.24 + 7.17, -0.72), # (3.93, -0.72)
    (-2.15 + 7.17, 0.00),  # (5.02, 0.0)
    (-3.24 + 7.17, 0.72),  # (3.93, 0.72)
    (-6.31 + 7.17, 0.72),  # (0.86, 0.72)
    (0.0, 0.0),
]
V_LIN = 0.6          # 线速度 (m/s)
GOAL_TOL = 0.2       # 到点阈值 (m), 适当放大一点以保证平滑过渡
RATE_HZ = 30

# --- 新增：控制器增益 ---
K_YAW = 2.0          # 航向误差的P增益
K_CTE = 3.0          # 横向误差的P增益
MAX_ANGULAR_SPEED = 2.0 # rad/s, 最大角速度限制

# -----------------------------

class StraightLineLoop(Node):
    def __init__(self):
        super().__init__('straight_line_loop_corrected')
        self.idx = 0
        self.x = self.y = self.yaw = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # 对于Odom，BEST_EFFORT更常用
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10) # 发布者用默认QoS即可
        self.timer = self.create_timer(1.0 / RATE_HZ, self.control_loop)

        self.get_logger().info('Waiting for /odom ...')

    def odom_cb(self, msg: Odometry):
        # ... (odom_cb 逻辑不变) ...
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not hasattr(self, 'odom_received'):
            self.get_logger().info('Start straight-line loop ...')
            self.odom_received = True

    def control_loop(self):
        if not hasattr(self, 'odom_received'):
            return

        # 当前段起点、终点
        # 使用 numpy 数组方便进行向量运算
        p_start = np.array(WAYPOINTS[self.idx])
        p_end = np.array(WAYPOINTS[(self.idx + 1) % len(WAYPOINTS)])
        p_robot = np.array([self.x, self.y])

        # 到终点的距离
        dist_to_goal = np.linalg.norm(p_end - p_robot)

        # 判断是否到达当前段终点
        if dist_to_goal < GOAL_TOL:
            self.get_logger().info(f'Reached waypoint {(self.idx + 1) % len(WAYPOINTS)}')
            self.idx = (self.idx + 1) % len(WAYPOINTS)
            return

        # --- 核心修改：融合控制器 ---

        # 1. 计算航向误差 (err_yaw)
        # 期望航向角（直线方向）
        path_vector = p_end - p_start
        target_yaw = math.atan2(path_vector[1], path_vector[0])
        err_yaw = target_yaw - self.yaw
        # 归一化到 [-pi, pi]
        err_yaw = math.atan2(math.sin(err_yaw), math.cos(err_yaw))

        # 2. 计算横向误差 (cross_track_error)
        # 使用向量叉乘的几何意义来计算点到直线的距离
        # 向量 a = p_end - p_start (路径向量)
        # 向量 b = p_robot - p_start (机器人相对起点向量)
        # 横向误差 = |a x b| / |a|
        path_vec_normalized = path_vector / np.linalg.norm(path_vector)
        robot_vec = p_robot - p_start
        cross_track_error = np.cross(path_vec_normalized, robot_vec)

        # 3. 融合控制
        # 航向修正分量
        w_yaw = K_YAW * err_yaw
        # 横向误差修正分量：使用atan来平滑控制，防止修正过猛
        # 当机器人偏离路径时，需要一个额外的转向来“切回”路径
        w_cte = math.atan2(K_CTE * cross_track_error, V_LIN)
        
        # 总角速度 = 航向修正 + 横向修正
        w = w_yaw + w_cte
        
        # 限制最大角速度
        w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, w))

        cmd = Twist()
        cmd.linear.x = V_LIN
        cmd.angular.z = w
        self.pub.publish(cmd)
        
        # 调试信息
        self.get_logger().info(f'CTE: {cross_track_error:.2f}, ErrYaw: {math.degrees(err_yaw):.1f}, W_CTE: {w_cte:.2f}, W_YAW: {w_yaw:.2f}, W_Total: {w:.2f}', throttle_duration_sec=0.2)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 在退出前发布停止指令
        stop_cmd = Twist()
        node.pub.publish(stop_cmd)
        node.get_logger().info('Node is shutting down, stopping the robot.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()