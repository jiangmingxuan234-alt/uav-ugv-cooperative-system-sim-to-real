#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraSubscriber:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('camera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rflysim/sensor2/img_rgb", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 显示图像
        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        camera_subscriber = CameraSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
