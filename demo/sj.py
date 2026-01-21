#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class MissionController:
    def __init__(self):
        rospy.init_node('integrated_mission_controller_final', anonymous=True)

        # =================================================================
        # ==                  1. 任务参数总览                          ==
        # =================================================================
        self.waypoints = [[0, 0, 1.2], [0, 1.8, 1.2]]
        self.waypoint_tolerance = 0.2
        self.CANNY_THRESH_1 = 50
        self.CANNY_THRESH_2 = 150
        self.MIN_CONTOUR_AREA = 500
        self.MIN_ASPECT_RATIO = 0.7 
        self.MAX_ASPECT_RATIO = 1.4
        
        self.IMG_WIDTH = 640
        self.IMG_HEIGHT = 480
        self.image_center_x = self.IMG_WIDTH / 2.0
        self.image_center_y = self.IMG_HEIGHT / 2.0

        self.FORWARD_SPEED = 0.3
        self.TARGET_DISTANCE = 4.1
        
        # --- 精准通行与稳定控制参数 ---
        self.AIM_POINT_VERTICAL_RATIO = 0.65
        self.KP_YAW = 0.005
        self.KP_Z = 0.004
        self.KD_Z = 0.003
        self.MAX_VERTICAL_SPEED = 0.15
        self.VERTICAL_DEADBAND = 10

        # =================================================================
        # ==                2. 状态机与ROS接口设置                     ==
        # =================================================================
        self.mission_state = "SETUP"
        self.current_mavros_state = State()
        self.current_pose = PoseStamped()
        self.tracker = None
        self.move_start_time = None
        self.last_error_y = 0.0
        self.last_time_y = rospy.Time.now()

        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_position_cb)
        rospy.Subscriber("/rflysim/sensor1/img_rgb", Image, self.image_callback)
        
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)
        rospy.loginfo("集成任务控制器已初始化 (V5.1 - 修正版)。")

    def state_cb(self, msg): self.current_mavros_state = msg
    def local_position_cb(self, msg): self.current_pose = msg

    def detect_initial_target(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.CANNY_THRESH_1, self.CANNY_THRESH_2)
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) < 2: return None
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
        if cv2.contourArea(contours[0]) < self.MIN_CONTOUR_AREA or cv2.contourArea(contours[1]) < self.MIN_CONTOUR_AREA: return None
        x1, y1, w1, h1 = cv2.boundingRect(contours[0]); x2, y2, w2, h2 = cv2.boundingRect(contours[1])
        center_x1 = x1 + w1 / 2.0; center_x2 = x2 + w2 / 2.0
        if abs(center_x1 - center_x2) > self.IMG_WIDTH * 0.7: return None
        x_left = min(x1, x2); y_top = min(y1, y2); x_right = max(x1 + w1, x2 + w2); y_bottom = max(y1 + h1, y2 + h2)
        box_width = float(x_right - x_left); box_height = float(y_bottom - y_top)
        if box_height > 0:
            aspect_ratio = box_width / box_height
            if self.MIN_ASPECT_RATIO < aspect_ratio < self.MAX_ASPECT_RATIO:
                return (x_left, y_top, int(box_width), int(box_height))
        return None

    def image_callback(self, data):
        if self.mission_state not in ["VISION_SEARCH", "VISION_TRACK_AND_MOVE"]: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.resize(cv_image, (self.IMG_WIDTH, self.IMG_HEIGHT))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge 错误: {e}"); return
        
        debug_image = cv_image.copy()
        twist_msg = Twist()
        
        if self.mission_state == "VISION_SEARCH":
            initial_bbox = self.detect_initial_target(cv_image)
            if initial_bbox:
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(cv_image, initial_bbox)
                self.move_start_time = rospy.Time.now(); self.last_time_y = rospy.Time.now(); self.last_error_y = 0.0
                self.mission_state = "VISION_TRACK_AND_MOVE"
                rospy.loginfo(">>> 视觉目标已找到！切换到 [VISION_TRACK_AND_MOVE] 状态。 <<<")
            self.cmd_vel_pub.publish(twist_msg)
        
        elif self.mission_state == "VISION_TRACK_AND_MOVE":
            elapsed_time = (rospy.Time.now() - self.move_start_time).to_sec()
            distance_traveled = elapsed_time * self.FORWARD_SPEED
            if distance_traveled >= self.TARGET_DISTANCE:
                self.mission_state = "MISSION_COMPLETE"
                rospy.loginfo(">>> 已飞行指定距离。切换到 [MISSION_COMPLETE] 状态。 <<<")
                self.cmd_vel_pub.publish(twist_msg); return

            success, bbox = self.tracker.update(cv_image)
            if success:
                p1 = (int(bbox[0]), int(bbox[1])); p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(debug_image, p1, p2, (0, 255, 0), 2, 1)
                
                aim_point_x = bbox[0] + bbox[2] / 2.0
                aim_point_y = bbox[1] + bbox[3] * self.AIM_POINT_VERTICAL_RATIO
                cv2.circle(debug_image, (int(aim_point_x), int(aim_point_y)), 7, (0, 0, 255), -1)

                error_x = aim_point_x - self.image_center_x
                error_y = self.image_center_y - aim_point_y

                current_time = rospy.Time.now(); dt = (current_time - self.last_time_y).to_sec()
                if dt > 0.001:
                    error_derivative_y = (error_y - self.last_error_y) / dt
                    vertical_speed_pd = self.KP_Z * error_y + self.KD_Z * error_derivative_y
                else: vertical_speed_pd = self.KP_Z * error_y
                self.last_error_y = error_y; self.last_time_y = current_time
                
                twist_msg.angular.z = -self.KP_YAW * error_x; twist_msg.linear.x = self.FORWARD_SPEED
                if abs(error_y) > self.VERTICAL_DEADBAND:
                    twist_msg.linear.z = np.clip(vertical_speed_pd, -self.MAX_VERTICAL_SPEED, self.MAX_VERTICAL_SPEED)
                else: twist_msg.linear.z = 0

                self.cmd_vel_pub.publish(twist_msg)
            else:
                rospy.logerr("追踪器失败！返回 [VISION_SEARCH] 状态。")
                self.mission_state = "VISION_SEARCH"; self.cmd_vel_pub.publish(twist_msg)
        
        cv2.imshow("视觉调试窗口", debug_image); cv2.waitKey(1)
        
    # =================================================================
    # ==               【修复】将此函数添加回来                      ==
    # =================================================================
    def distance_to_target(self, target_pos):
        """计算无人机当前位置到目标点(x,y,z)的距离"""
        dx = self.current_pose.pose.position.x - target_pos[0]
        dy = self.current_pose.pose.position.y - target_pos[1]
        dz = self.current_pose.pose.position.z - target_pos[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def run(self):
        # ... 主循环 run() 函数无需修改 ...
        while not rospy.is_shutdown() and not self.current_mavros_state.connected:
            rospy.loginfo_throttle(2.0, "等待飞控(FCU)连接...")
            self.rate.sleep()
        pose = PoseStamped(); pose.pose.position.x = self.waypoints[0][0]; pose.pose.position.y = self.waypoints[0][1]; pose.pose.position.z = self.waypoints[0][2]
        for i in range(100):
            if rospy.is_shutdown(): break
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
        offb_set_mode = SetModeRequest(custom_mode='OFFBOARD'); arm_cmd = CommandBoolRequest(value=True); last_req = rospy.Time.now(); current_waypoint_index = 0
        rospy.loginfo("启动主任务循环。")
        while not rospy.is_shutdown():
            if self.current_mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent: rospy.loginfo("OFFBOARD 模式已启用")
                last_req = rospy.Time.now()
            elif not self.current_mavros_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.arming_client.call(arm_cmd).success: rospy.loginfo("无人机已解锁")
                last_req = rospy.Time.now()
            if self.mission_state == "SETUP":
                if self.current_mavros_state.mode == "OFFBOARD" and self.current_mavros_state.armed:
                    self.mission_state = "WAYPOINT_NAV"
                    rospy.loginfo(">>> 设置完成。切换到 [WAYPOINT_NAV] 状态。 <<<")
                self.local_pos_pub.publish(pose)
            elif self.mission_state == "WAYPOINT_NAV":
                current_target_pos = self.waypoints[current_waypoint_index]
                pose.pose.position.x = current_target_pos[0]; pose.pose.position.y = current_target_pos[1]; pose.pose.position.z = current_target_pos[2]
                self.local_pos_pub.publish(pose)

                # 这一行现在可以正常工作了
                if self.distance_to_target(current_target_pos) < self.waypoint_tolerance:
                    rospy.loginfo(f"已到达航点 {current_waypoint_index + 1}。")
                    if current_waypoint_index < len(self.waypoints) - 1:
                        current_waypoint_index += 1
                    else:
                        rospy.loginfo(">>> 已到达最终航点。即将切换到速度控制模式。 <<<")
                        for i in range(10): self.cmd_vel_pub.publish(Twist()); self.rate.sleep()
                        self.mission_state = "VISION_SEARCH"
                        rospy.loginfo(">>> 切换到 [VISION_SEARCH] 状态。 <<<")
            elif self.mission_state in ["VISION_SEARCH", "VISION_TRACK_AND_MOVE"]: pass
            elif self.mission_state == "MISSION_COMPLETE":
                rospy.loginfo_throttle(5.0, "任务完成。保持悬停。")
                self.cmd_vel_pub.publish(Twist()) 
            self.rate.sleep()


if __name__ == "__main__":
    try:
        controller = MissionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("任务节点已关闭。")