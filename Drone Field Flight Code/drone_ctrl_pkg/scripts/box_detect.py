#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class BoxDetectorNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('box_detector_node', anonymous=True)
        # 创建 CvBridge 对象，用于在 ROS 图像消息和 OpenCV 图像之间转换
        self.bridge = CvBridge()
        # 订阅无人机的摄像头图像话题
        self.image_sub = rospy.Subscriber("/drone/camera/image_raw", Image, self.image_callback)
        # 发布方框中心位置话题
        self.center_pub = rospy.Publisher("/box_center", Point, queue_size=10)

    def image_callback(self, data):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s", e)
            return

        # 转换为灰度图像
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 使用 Canny 算子进行边缘检测
        edges = cv2.Canny(gray, 50, 150)
        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        box_center = None
        for contour in contours:
            # 计算轮廓的周长
            perimeter = cv2.arcLength(contour, True)
            # 对轮廓进行多边形近似
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            # 如果轮廓有 4 个顶点，并且轮廓面积大于一定阈值，则认为是中空方框
            if len(approx) == 4 and cv2.contourArea(contour) > 1000:
                # 计算轮廓的边界框
                x, y, w, h = cv2.boundingRect(contour)
                # 计算方框中心坐标
                center_x = x + w / 2
                center_y = y + h / 2
                box_center = (center_x, center_y)
                # 在图像上绘制方框的边界框
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # 在图像上绘制方框中心点
                cv2.circle(cv_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                break

        if box_center is not None:
            # 发布方框中心位置信息
            center_msg = Point()
            center_msg.x = box_center[0]
            center_msg.y = box_center[1]
            center_msg.z = 0  # 如果需要，可以添加 z 坐标信息
            self.center_pub.publish(center_msg)

        # 显示处理后的图像（可选）
        cv2.imshow("Detected Box", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        box_detector = BoxDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass