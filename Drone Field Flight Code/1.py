#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Float64MultiArray

class PoseControllerNode:
    def __init__(self):
        """
        初始化节点、订阅者、发布者和服务客户端。
        """
        # 初始化 ROS 节点
        rospy.init_node('pose_ctrl_node_py', anonymous=True)

        # --- 状态变量 ---
        self.current_state = None
        self.current_pose = PoseStamped()
        
        # --- 设定点 Pose ---
        self.pose = PoseStamped()
        # 初始起飞高度
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2.0  # 默认起飞高度
        self.pose.pose.orientation.w = 1.0

        # --- ROS 通信接口 ---
        # 订阅 MAVROS 状态
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        
        # 订阅当前本地位置
        self.current_pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.current_pose_cb)

        # 订阅用于调整姿态的命令
        self.alter_pose_sub = rospy.Subscriber(
            "alter_pose_cmd", Float64MultiArray, self.alter_pose_cb)

        # 订阅用于设置绝对姿态的命令
        self.set_pose_sub = rospy.Subscriber(
            "set_pose_cmd", Float64MultiArray, self.set_pose_cb)
            
        # 发布设定点位置
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # 等待 MAVROS 服务可用
        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.loginfo("MAVROS services available.")

        # 服务客户端
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # 设置循环频率
        self.rate = rospy.Rate(20.0)

    # --- 回调函数 ---
    def state_cb(self, msg):
        self.current_state = msg

    def alter_pose_cb(self, msg):
        if len(msg.data) == 3:
            dx, dy, dz = msg.data
            self.pose.pose.position.x += dx
            self.pose.pose.position.y += dy
            self.pose.pose.position.z += dz
            rospy.loginfo("Received pose adjustment: dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz)
        else:
            rospy.logwarn("alter_pose_cmd requires 3 values (dx, dy, dz).")

    def set_pose_cb(self, msg):
        if len(msg.data) == 3:
            x, y, z = msg.data
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = z
            rospy.loginfo("Moving pose to x=%.2f, y=%.2f, z=%.2f", x, y, z)
        else:
            rospy.logwarn("set_pose_cmd requires 3 values (x, y, z).")


    def current_pose_cb(self, msg):
        self.current_pose = msg

    # --- 核心逻辑函数 ---
    def move_to(self, dx, dy, dz, steps=100):
        """
        相对于当前设定点移动一个增量。
        """
        rospy.loginfo("Moving by dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz)
        self.pose.pose.position.x += dx
        self.pose.pose.position.y += dy
        self.pose.pose.position.z += dz
        for _ in range(steps):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
            
    def run(self):
        """
        主执行循环。
        """
        # 等待与飞控建立连接
        rospy.loginfo("Waiting for FCU connection...")
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            self.rate.sleep()
        rospy.loginfo("FCU connected.")

        # 在进入 OFFBOARD 模式前，必须先发送设定点
        rospy.loginfo("Publishing setpoints before entering OFFBOARD mode...")
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        # 设置请求对象
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_request_time = rospy.Time.now()

        rospy.loginfo("Attempting to enter OFFBOARD mode and arm...")
        # 循环直到进入 OFFBOARD 模式并成功解锁
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request_time > rospy.Duration(5.0)):
                try:
                    if self.set_mode_client.call(offb_set_mode).mode_sent:
                        rospy.loginfo("OFFBOARD mode enabled")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
                last_request_time = rospy.Time.now()
            elif not self.current_state.armed and (rospy.Time.now() - last_request_time > rospy.Duration(5.0)):
                try:
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
                last_request_time = rospy.Time.now()

            # 如果已经成功，则跳出循环
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                rospy.loginfo("Vehicle is in OFFBOARD mode and armed.")
                break

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        rospy.loginfo("Starting autonomous mission...")
        
        # -------- 自动飞行路径 --------
        # 注意：C++代码中的 `move_to(0, 2，0);` 有一个全角逗号，这里已修正。
        self.move_to(0, 0, 0)      # 保持在 (0, 0, 2.0) 起飞
        self.move_to(0, 2, 0)      # 移动到 (0, 2, 2.0)
        # self.move_to(4.0, 0, 0)    # 移动到 (4, 2, 2.0)
        # self.move_to(0, -1.5, 0)   # 移动到 (4, 0.5, 2.0)
        # self.move_to(-2.0, 0, 0)   # 移动到 (2, 0.5, 2.0)
        # self.move_to(0, -1.45, 0)  # 移动到 (2, -0.95, 2.0)

        # 慢慢下降至 0.5 米
        rospy.loginfo("Descending to 0.5m...")
        current_z = self.pose.pose.position.z
        while not rospy.is_shutdown() and current_z > 0.5:
            current_z -= 0.05
            self.pose.pose.position.z = current_z
            for _ in range(5):  # 每个高度维持约 0.25s
                if rospy.is_shutdown():
                    break
                self.local_pos_pub.publish(self.pose)
                self.rate.sleep()
            if rospy.is_shutdown():
                break

        rospy.loginfo("Hovering at final target position...")

        # -------- 悬停循环 --------
        while not rospy.is_shutdown():
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = PoseControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")