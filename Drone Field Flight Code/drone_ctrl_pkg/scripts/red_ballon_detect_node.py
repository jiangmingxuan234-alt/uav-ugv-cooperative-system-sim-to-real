#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class BalloonDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.image_callback)
        self.balloon_pub = rospy.Publisher("/balloon_position", Point, queue_size=10)
        self.lower_red = np.array([0, 120, 70])
        self.upper_red = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print(cv_image.shape)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        balloon_position = Point()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # 筛选面积较大的轮廓
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                center_x = x + w / 2
                center_y = y + h / 2
                balloon_position.x = center_x
                balloon_position.y = center_y
                balloon_position.z = 0.0  # 可选择添加深度信息

        self.balloon_pub.publish(balloon_position)

        cv2.imshow("Image", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('balloon_detector')
    detector = BalloonDetector()
    rospy.spin()