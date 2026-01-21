#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无限循环直线轨迹跟踪（ROS 2）
运行：
    ros2 run <pkg> straight_line_loop
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ----------- 参数 -----------
WAYPOINTS = [
    (0, 0.00),
    (-6.31 + 7.17, -0.72),
    (-3.24 + 7.17, -0.72),
    (-2.15 + 7.17, 0.00),
    (-3.24 + 7.17, 0.72),
    (-6.31 + 7.17, 0.72),
    (0, 0.00),
]
V_LIN = 0.6          # 线速度 (m/s)
GOAL_TOL = 0.1       # 到点阈值 (m)
RATE_HZ = 30
# -----------------------------

class StraightLineLoop(Node):
    def __init__(self):
        super().__init__('straight_line_loop')
        self.idx = 0
        self.x = self.y = self.yaw = 0.0

        # QoS 与 ROS1 默认保持一致
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.control_loop)

        self.get_logger().info('Waiting for /odom ...')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 一旦收到有效里程计，就打印一次
        if not hasattr(self, 'odom_received'):
            self.get_logger().info('Start straight-line loop ...')
            self.odom_received = True

    def control_loop(self):
        if not hasattr(self, 'odom_received'):
            return  # 尚未收到里程计，不控制

        # 当前段起点、终点
        x0, y0 = WAYPOINTS[self.idx]
        x1, y1 = WAYPOINTS[(self.idx + 1) % len(WAYPOINTS)]

        # 到终点的距离
        dx, dy = x1 - self.x, y1 - self.y
        dist = math.hypot(dx, dy)

        # 判断是否到达当前段终点
        if dist < GOAL_TOL:
            self.get_logger().info(
                f'Reached waypoint {(self.idx + 1) % len(WAYPOINTS)}')
            self.idx = (self.idx + 1) % len(WAYPOINTS)
            return

        # 期望航向角（直线方向）
        target_yaw = math.atan2(dy, dx)
        err_yaw = target_yaw - self.yaw
        err_yaw = math.atan2(math.sin(err_yaw), math.cos(err_yaw))

        # P 控制
        w = 2.0 * err_yaw
        w = max(-2.0, min(2.0, w))

        cmd = Twist()
        cmd.linear.x = V_LIN
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()