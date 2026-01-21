#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ===============================================================
#                 代码使用的常量 (请在这里调整！)
# ===============================================================
NODE_RATE = 20.0     # Hz, 节点主循环的频率，保证高频发布指令
FORWARD_SPEED = 0.22  # m/s, 前进速度
TURN_SPEED = 1.0     # rad/s, 转向速度的绝对值
TURN_FORWARD_SPEED = 0.05 # m/s, 避障时给的微小前进速度

# --- 阿克曼特性关键常量 ---
U_TURN_FORWARD_SPEED = 0.1 # m/s, 阿克曼掉头时的前进速度，必须不为0！

OBSTACLE_DISTANCE_THRESHOLD = 0.45 # m, 机器人正前方多近算作障碍物
MAX_AVOID_TIME_S = 5.0 # s, 最大避障转向时间

# --- 常量：用于第二次避障后的特殊行为 ---
POST_AVOID_DISTANCE = 1.0    # m, 第二次避障后直行的距离
PRE_TURN_PAUSE_S = 2.0       # s, 掉头前暂停的时间
# --- 核心修改：使用固定的掉头时间，需要手动标定 ---
# 初始值可以给大一点，比如8.0秒，然后根据实际效果调整
U_TURN_DURATION_S = 10.0     # s, 阿克曼掉头的总时长

class PhaseReactiveNavigator(Node):
    def __init__(self):
        super().__init__('phase_reactive_navigator')
        
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.timer = self.create_timer(1.0 / NODE_RATE, self.main_loop)

        self.state = 'GO_STRAIGHT' 
        self.state_transition_time = 0.0
        self.encounter_phase = 1
        self.turn_direction = 0.0
        
        self.latest_scan_msg = None
        self.min_front_dist = float('inf')

        self.get_logger().info("阿克曼导航节点已启动。")
        self.log_state()

    def log_state(self):
        self.get_logger().info(f"当前状态: {self.state}, 遭遇阶段: {self.encounter_phase}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan_msg = msg

    def process_scan_data(self):
        if self.latest_scan_msg is None: return
        msg = self.latest_scan_msg
        num_readings = len(msg.ranges)
        if num_readings == 0: self.min_front_dist = float('inf'); return
        center_index = num_readings // 2
        angle_increment = msg.angle_increment
        if angle_increment <= 0: self.min_front_dist = float('inf'); return
        half_index_range = int((math.radians(30.0) / angle_increment) / 2)
        front_readings_valid = [r for r in msg.ranges[center_index-half_index_range : center_index+half_index_range] if math.isfinite(r) and r > 0.01]
        self.min_front_dist = min(front_readings_valid) if front_readings_valid else float('inf')

    def main_loop(self):
        self.process_scan_data()
        
        twist_msg = Twist() # 默认是停止指令
        
        # ... (GO_STRAIGHT, AVOIDING_OBSTACLE, POST_AVOID_STRAIGHT, PRE_TURN_PAUSE 状态逻辑不变) ...
        if self.state == 'GO_STRAIGHT':
            if self.min_front_dist < OBSTACLE_DISTANCE_THRESHOLD:
                self.state = 'AVOIDING_OBSTACLE'
                self.state_transition_time = time.time()
                if self.encounter_phase == 1:
                    self.get_logger().info(f"障碍物距离: {self.min_front_dist:.2f}m. 遭遇阶段1 -> 开始右转")
                    self.turn_direction = -1.0
                    self.encounter_phase = 2
                else:
                    self.get_logger().info(f"障碍物距离: {self.min_front_dist:.2f}m. 遭遇阶段2 -> 开始左转")
                    self.turn_direction = 1.0
                    self.encounter_phase = 1
                self.log_state()
            else:
                twist_msg.linear.x = FORWARD_SPEED

        elif self.state == 'AVOIDING_OBSTACLE':
            exit_condition = (self.min_front_dist > OBSTACLE_DISTANCE_THRESHOLD * 1.5) or \
                             (time.time() - self.state_transition_time > MAX_AVOID_TIME_S)
            if exit_condition:
                if self.encounter_phase == 1: 
                    self.get_logger().info("第二次避障完成。切换到 -> POST_AVOID_STRAIGHT")
                    self.state = 'POST_AVOID_STRAIGHT'
                else: 
                    self.get_logger().info("第一次避障完成。切换回 -> GO_STRAIGHT")
                    self.state = 'GO_STRAIGHT'
                self.state_transition_time = time.time()
                self.log_state()
            else:
                twist_msg.linear.x = TURN_FORWARD_SPEED
                twist_msg.angular.z = self.turn_direction * TURN_SPEED

        elif self.state == 'POST_AVOID_STRAIGHT':
            drive_duration = POST_AVOID_DISTANCE / FORWARD_SPEED
            if time.time() - self.state_transition_time > drive_duration:
                self.get_logger().info(f"直行 {POST_AVOID_DISTANCE}m 完成。切换到 -> PRE_TURN_PAUSE")
                self.state = 'PRE_TURN_PAUSE'
                self.state_transition_time = time.time()
                self.log_state()
            else:
                twist_msg.linear.x = FORWARD_SPEED
        
        elif self.state == 'PRE_TURN_PAUSE':
            if time.time() - self.state_transition_time > PRE_TURN_PAUSE_S:
                self.get_logger().info(f"暂停 {PRE_TURN_PAUSE_S}s 完成。切换到 -> U_TURN")
                self.state = 'U_TURN'
                self.state_transition_time = time.time()
                self.log_state()

        elif self.state == 'U_TURN':
            # --- 核心修改：使用固定的时间来控制掉头 ---
            if time.time() - self.state_transition_time > U_TURN_DURATION_S:
                self.get_logger().info(f"掉头 {U_TURN_DURATION_S}s 完成。切换回 -> GO_STRAIGHT")
                self.state = 'GO_STRAIGHT'
                self.log_state()
            else:
                # 保持前进并左转的指令
                twist_msg.linear.x = U_TURN_FORWARD_SPEED
                twist_msg.angular.z = TURN_SPEED 
        
        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhaseReactiveNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.velocity_publisher.publish(stop_msg)
        node.get_logger().info('节点关闭，机器人已停止。')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()