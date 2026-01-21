#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # 导入用于发布速度指令的消息类型
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BoxNavigator:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('box_navigator', anonymous=True)

        # --- 可调参数 ---

        # 1. 颜色范围 (HSV)
        # 这是一个示例，用于检测蓝色。你需要根据你的实际目标颜色进行调整。
        # 你可以使用任何HSV颜色选择器工具来找到合适的范围。
        # H: 0-179, S: 0-255, V: 0-255 in OpenCV
        self.hsv_lower = np.array([100, 150, 50])  # HSV下限 (蓝色)
        self.hsv_upper = np.array([130, 255, 255]) # HSV上限 (蓝色)

        # 2. 控制器增益 (P控制器)
        # Kp值越大，对误差的反应越快，但也可能导致不稳定/震荡。从小的值开始调。
        self.KP_YAW = 0.005   # 偏航角速度的比例增益 (左右)
        self.KP_Z = 0.008     # Z轴线速度的比例增益 (上下)

        # 3. 目标速度
        self.FORWARD_SPEED = 0.3  # 恒定的前进速度 (米/秒)

        # 4. ROS 话题
        image_topic = "/rflysim/sensor1/img_rgb" # 图像订阅话题
        velocity_topic = "/cmd_vel"             # 速度发布话题 (根据你的无人机配置可能需要修改)

        # --- 初始化 ---
        self.bridge = CvBridge()
        self.target_found = False

        # 初始化速度发布者
        self.velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size=10)

        # 初始化图像订阅者 (放在最后，确保其他都初始化好了)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        rospy.loginfo("Box Navigator Node Started. Waiting for images...")

    def image_callback(self, data):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 获取图像尺寸
        h, w, d = cv_image.shape
        image_center_x = w / 2
        image_center_y = h / 2

        # --- 视觉处理 ---
        # 1. 转换到 HSV 颜色空间
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. 根据颜色范围创建掩码
        mask = cv2.inRange(hsv_image, self.hsv_lower, self.hsv_upper)
        
        # (可选) 对掩码进行形态学操作，以消除噪声
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # 3. 寻找掩码中的轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.target_found = False
        if len(contours) > 0:
            # 找到最大的轮廓 (假设最大的轮廓就是我们的目标)
            c = max(contours, key=cv2.contourArea)
            # 获取轮廓的矩形边界
            x, y, w_box, h_box = cv2.boundingRect(c)

            # 只有当检测到的框足够大时才认为找到了目标，以防噪声干扰
            if w_box > 30 and h_box > 30:
                self.target_found = True
                
                # 计算目标中心
                target_center_x = x + w_box / 2
                target_center_y = y + h_box / 2

                # --- 在图像上绘制信息，用于调试 ---
                cv2.rectangle(cv_image, (x, y), (x + w_box, y + h_box), (0, 255, 0), 2)
                cv2.circle(cv_image, (int(target_center_x), int(target_center_y)), 5, (0, 0, 255), -1)
                cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 5, (255, 0, 0), -1)
                cv2.line(cv_image, (int(target_center_x), int(target_center_y)), (int(image_center_x), int(image_center_y)), (0,0,0), 2)

                # --- 控制逻辑 ---
                # 计算目标中心与图像中心的误差
                error_x = target_center_x - image_center_x
                error_y = image_center_y - target_center_y # Y轴方向相反

                # 创建 Twist 消息
                twist_msg = Twist()

                # 根据误差计算控制量
                # 左右转向 (控制偏航角速度 angular.z)
                # 注意：ROS标准中，angular.z为正值表示逆时针旋转（向左转）
                twist_msg.angular.z = -self.KP_YAW * error_x
                
                # 上下移动 (控制Z轴线速度 linear.z)
                # 注意：ROS标准中，linear.z为正值表示向上移动
                twist_msg.linear.z = self.KP_Z * error_y
                
                # 保持前进
                twist_msg.linear.x = self.FORWARD_SPEED

                # 发布速度指令
                self.velocity_publisher.publish(twist_msg)

        if not self.target_found:
            # 如果没有找到目标，发送悬停指令 (所有速度为0)
            rospy.logwarn_throttle(1.0, "Target not found. Hovering.")
            hover_msg = Twist()
            self.velocity_publisher.publish(hover_msg)

        # 显示处理后的图像
        cv2.imshow("Navigation View", cv_image)
        # cv2.imshow("Mask", mask) # 取消注释以查看掩码效果
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        navigator = BoxNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutting down.")
    finally:
        # 确保在退出时关闭所有OpenCV窗口
        cv2.destroyAllWindows()