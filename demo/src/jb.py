# main.py (修改版：右移 -> 无限悬停，按Ctrl+C降落)

import cv2
import numpy as np
import time
import math
import sys

# 导入 RflySim 的核心库
import VisionCaptureApi
import ReqCopterSim
import RflyRosStart

# 导入 ROS 和 MAVROS 相关库
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, SetMode
except ImportError:
    print("错误: 无法导入 ROS/MAVROS 库。")
    sys.exit(1)

# ==============================================================================
# 0. 全局变量和回调函数
# ==============================================================================
current_state = State()
current_pos = PoseStamped()

def state_cb(msg): global current_state; current_state = msg
def pos_cb(msg): global current_pos; current_pos = msg

# ==============================================================================
# 1. 基础飞行与任务函数
# ==============================================================================
def set_mode(mode="OFFBOARD"):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        if set_mode_client(custom_mode=mode).mode_sent:
            print(f"飞行模式已设置为 {mode}"); return True
    except rospy.ServiceException as e:
        print(f"模式设置服务调用失败: {e}"); return False

def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        if arming_client(True).success:
            print("无人机已解锁"); return True
    except rospy.ServiceException as e:
        print(f"解锁服务调用失败: {e}"); return False

def arm_and_takeoff_by_pos(altitude, pos_pub, rate):
    """使用位置控制器起飞"""
    print("等待 MAVROS 连接...")
    while not current_state.connected and not rospy.is_shutdown(): rate.sleep()
    print("准备起飞...")
    target_pose = PoseStamped()
    target_pose.pose.position.x = 0; target_pose.pose.position.y = 0; target_pose.pose.position.z = altitude
    for i in range(100):
        if rospy.is_shutdown(): break
        pos_pub.publish(target_pose)
        rate.sleep()
    if not set_mode("OFFBOARD"): return
    time.sleep(1)
    if not arm(): return
    print(f"起飞到 {altitude} 米...")
    while not rospy.is_shutdown():
        pos_pub.publish(target_pose)
        if abs(current_pos.pose.position.z - altitude) < 0.2:
            print("已达到目标高度，切换为速度控制模式。")
            break
        rate.sleep()

def send_vel_cmd(vx, vy, vz, avx, avy, avz, duration, vel_pub, rate):
    """通用速度指令发送函数 (使用 cmd_vel 并指定机体坐标系)"""
    vel_cmd = TwistStamped()
    end_time = time.time() + duration
    while time.time() < end_time and not rospy.is_shutdown():
        vel_cmd.header.frame_id = "base_link" 
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.twist.linear.x = vx; vel_cmd.twist.linear.y = vy; vel_cmd.twist.linear.z = vz
        vel_cmd.twist.angular.x = avx; vel_cmd.twist.angular.y = avy; vel_cmd.twist.angular.z = avz
        vel_pub.publish(vel_cmd)
        rate.sleep()

def hover(duration, vel_pub, rate):
    """悬停（刹车）函数，非常重要！"""
    # 修改：仅在持续时间较长时打印，避免在循环中刷屏
    if duration > 0.1:
        print(f"悬停稳定 {duration} 秒...")
    send_vel_cmd(0, 0, 0, 0, 0, 0, duration, vel_pub, rate)

def turn_by_vel(angle_degrees, vel_pub, rate):
    """使用速度控制器进行旋转"""
    print(f"准备旋转 {angle_degrees} 度...")
    yaw_speed_rad = 0.5 # rad/s
    direction = 1 if angle_degrees > 0 else -1 # 正为左转, 负为右转
    duration = abs(math.radians(angle_degrees)) / yaw_speed_rad
    send_vel_cmd(0, 0, 0, 0, 0, direction * yaw_speed_rad, duration, vel_pub, rate)
    print("旋转完成。")

def move_right(distance_m, vel_pub, rate):
    """向右平移指定距离"""
    print(f"向右平移 {distance_m} 米...")
    speed_mps = 0.5 # m/s
    duration = abs(distance_m) / speed_mps
    # 使用线速度Y (vy) 控制右移
    send_vel_cmd(0, speed_mps, 0, 0, 0, 0, duration, vel_pub, rate)
    print("平移完成。")

# 注意：此函数定义依然保留，只是在主程序中不再调用它
def find_and_fly_through_box(vis, vel_pub, rate):
    # 函数内容保持不变，只是不会被调用
    pass

# ==============================================================================
# 3. 主程序入口
# ==============================================================================
if __name__ == '__main__':
    # --- RflySim 初始化 ---
    VisionCaptureApi.isEnableRosTrans = True; req = ReqCopterSim.ReqCopterSim(); StartCopterID = 1
    TargetIP = req.getSimIpID(StartCopterID)
    if TargetIP is None: sys.exit("错误: 未找到 CopterSim 实例。")
    req.sendReSimIP(StartCopterID)
    if not (RflyRosStart.isLinux and RflyRosStart.isRosOk): sys.exit('错误: 此程序必须在带ROS的Linux环境下运行')
    ros = RflyRosStart.RflyRosStart(StartCopterID,TargetIP)
    vis = VisionCaptureApi.VisionCaptureApi(TargetIP)
    vis.jsonLoad(); vis.sendReqToUE4(0, TargetIP); vis.startImgCap(); vis.sendImuReqCopterSim(StartCopterID,TargetIP)
    print('RflySim API 初始化完成')

    # --- ROS MAVROS 初始化 ---
    rospy.init_node('rfly_mission_node', anonymous=True)
    rate = rospy.Rate(20)
    rospy.Subscriber('/mavros/state', State, callback=state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=pos_cb)
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    print("等待获取无人机初始位置..."); time.sleep(2)
    
    try:
        # ==================== 任务流程开始 ====================
        
        # 任务1: 起飞并悬停
        arm_and_takeoff_by_pos(1.5, pos_pub, rate)
        hover(2, vel_pub, rate)

        # 任务2: 向右平移2米
        print("\n--- 任务2: 向右平移2米 ---")
        move_right(2, vel_pub, rate)
        hover(1, vel_pub, rate) # 动作后必须悬停稳定！

        # ########################################################
        # ##        任务4 (穿框) 已按要求修改为无限悬停         ##
        # ########################################################
        # print("\n--- 任务4: 识别并穿过方框 ---")
        # find_and_fly_through_box(vis, vel_pub, rate)
        # hover(1, vel_pub, rate) # 任务结束后悬停稳定！

        # ==================== 进入无限悬停模式 ====================
        print("\n--- 任务流程结束，进入无限悬停模式 ---")
        print(">>> 按下 Ctrl+C 来终止程序并自动降落 <<<")
        
        while not rospy.is_shutdown():
            # 持续发送短时悬停指令以保持飞机稳定
            hover(0.1, vel_pub, rate)

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        # 当用户按下 Ctrl+C 时，会捕获这个异常
        print("\n检测到用户中断(Ctrl+C)...")
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"任务执行中发生意外错误: {e}")
    finally:
        # 无论程序如何退出，最终都会执行这里的代码，实现安全降落
        print("\n程序终止，执行最后的安全操作：自动降落。")
        set_mode("AUTO.LAND")
        vis.stopImgCap()
        print("程序退出。")