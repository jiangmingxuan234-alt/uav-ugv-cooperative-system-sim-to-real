import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
import math
import time
import cv2
import numpy as np

current_state = State()
current_pose = PoseStamped()
camera_image = None  # 用于存储摄像头图像
red_ball_position = None  # 用于存储红色气球的中心位置

# 处理状态回调函数
def state_cb(msg):
    global current_state
    current_state = msg

# 处理本地位置回调函数
def local_position_cb(msg):
    global current_pose
    current_pose = msg

# 处理摄像头图像回调函数
def camera_cb(msg):
    global camera_image
    # 将 ROS 图像消息转换为 OpenCV 图像格式
    camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

# 计算当前位置与目标位置之间的距离
def distance_to_target(target):
    dx = current_pose.pose.position.x - target[0]
    dy = current_pose.pose.position.y - target[1]
    dz = current_pose.pose.position.z - target[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

# 检测红色气球并返回其中心位置
def detect_red_ball(image):
    # 将图像从 BGR 转换为 HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 定义红色的 HSV 范围
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # 创建红色的掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 使用掩膜提取红色区域
    red_area = cv2.bitwise_and(image, image, mask=mask)
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 如果检测到轮廓，计算轮廓的中心
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
    return None

# 计算速度指令
def calculate_velocity(image_center, ball_position, max_speed=0.5):
    # 计算偏差
    dx = ball_position[0] - image_center[0]
    dy = ball_position[1] - image_center[1]
    
    # 计算速度
    vx = dx / image_center[0] * max_speed
    vy = dy / image_center[1] * max_speed
    
    return vx, vy

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # 订阅 MAVROS 状态和本地位置话题
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=local_position_cb)
    # 订阅摄像头图像话题
    camera_sub = rospy.Subscriber("/rflysim/sensor2/img_rgb", Image, callback=camera_cb)

    # 发布速度指令
    velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    # 发布位置指令
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing 必须比 2Hz 更快
    rate = rospy.Rate(30)

    # 等待飞控连接
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()

    # 目标位置

    target1 = [7.13, 0.76, 1]
    targets = [target1]

    # 初始目标位置
    current_target = targets[0]
    pose.pose.position.x = current_target[0]
    pose.pose.position.y = current_target[1]
    pose.pose.position.z = current_target[2]

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    # 任务结束后的悬停时间（秒）
    hover_time = 1.0  # 悬停 10 秒

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
                for i in range(10):
                    if rospy.is_shutdown():
                        break
                    local_pos_pub.publish(pose)
                    rate.sleep()

            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        # 打印当前本地位置
        rospy.loginfo("当前本地位置: x = %.2f, y = %.2f, z = %.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)

        # 判断是否到达当前目标位置
        if distance_to_target(current_target) < 0.1:
            rospy.loginfo("已到达目标位置：%.2f, %.2f, %.2f", current_target[0], current_target[1], current_target[2])
            # 切换到下一个目标
            targets_index = targets.index(current_target)
            if targets_index < len(targets) - 1:
                current_target = targets[targets_index + 1]
                rospy.loginfo("切换到下一个目标：%.2f, %.2f, %.2f", current_target[0], current_target[1], current_target[2])
                pose.pose.position.x = current_target[0]
                pose.pose.position.y = current_target[1]
                pose.pose.position.z = current_target[2]
            else:
                rospy.loginfo("到达最终位置附近: x = %.2f, y = %.2f, z = %.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
                rospy.loginfo("任务完成，开始悬停...")
                start_hover_time = rospy.Time.now()
                while (rospy.Time.now() - start_hover_time).to_sec() < hover_time and not rospy.is_shutdown():
                    local_pos_pub.publish(pose)
                    rate.sleep()
                rospy.loginfo("悬停结束，开始追踪红色气球...")

                # 初始化速度指令
                velocity_cmd = TwistStamped()

                # 追踪红色气球
                while not rospy.is_shutdown():
                    if camera_image is not None:
                        red_ball_position = detect_red_ball(camera_image)
                        if red_ball_position:
                            rospy.loginfo("检测到红色气球，位置: (%d, %d)", red_ball_position[0], red_ball_position[1])
                            # 计算图像中心
                            image_center = (camera_image.shape[1] // 2, camera_image.shape[0] // 2)
                            # 计算速度指令
                            vx, vy = calculate_velocity(image_center, red_ball_position)
                            velocity_cmd.twist.linear.x = -vx*3.5
                            velocity_cmd.twist.linear.y = -vy*3.5
                            velocity_cmd.twist.linear.z = -1.05 # 保持高度不变
                        else:
                            rospy.loginfo("未检测到红色气球")
                            velocity_cmd.twist.linear.x = 0.0
                            velocity_cmd.twist.linear.y = 0.0
                            velocity_cmd.twist.linear.z = 0.0

                    # 发布速度指令
                    velocity_pub.publish(velocity_cmd)
                    rate.sleep()
                break

        rate.sleep()

    print("......................任务完成..............")