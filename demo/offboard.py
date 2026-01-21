#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import time

# --- 全局变量和回调函数部分保持不变 ---
# ... (此处省略与上一版本完全相同的代码) ...
# --- 全局变量 ---
current_state = State()
current_pose = PoseStamped()
STATE_WAYPOINTS = 1
STATE_SEARCH_GATE = 2
STATE_FLY_THROUGH_GATE = 3
STATE_DONE = 4
task_state = STATE_WAYPOINTS
gate_pose_target = None
gate_world_x_position = None

# --- 回调函数 ---
def state_cb(msg):
    global current_state
    current_state = msg

def local_position_cb(msg):
    global current_pose
    current_pose = msg

def lidar_cb(msg):
    global gate_pose_target, task_state, gate_world_x_position
    if task_state != STATE_SEARCH_GATE:
        return
    
    rospy.loginfo_once("开始处理雷达数据以搜索门框...")

    gate_points = []
    ROI = {
        'x_min': 0.5, 'x_max': 3.0,
        'y_min': -2.0, 'y_max': 2.0,
        'z_min': 0.1, 'z_max': 3.0
    }

    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        p_x, p_y, p_z = point[0], point[1], point[2]
        if (ROI['x_min'] < p_x < ROI['x_max'] and
            ROI['y_min'] < p_y < ROI['y_max'] and
            ROI['z_min'] < p_z < ROI['z_max']):
            gate_points.append(point)

    rospy.loginfo_throttle(2, "在ROI中找到 %d 个点 (需要 > 50)", len(gate_points))

    if len(gate_points) > 50:
        avg_x = sum([p[0] for p in gate_points]) / len(gate_points)
        y_coords = [p[1] for p in gate_points]
        z_coords = [p[2] for p in gate_points]
        y_min, y_max = min(y_coords), max(y_coords)
        z_min, z_max = min(z_coords), max(z_coords)
        center_y = (y_min + y_max) / 2.0
        center_z = (z_min + z_max) / 2.0
        
        rospy.loginfo("在雷达坐标系下发现门框中心: x=%.2f, y=%.2f, z=%.2f", avg_x, center_y, center_z)
        
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        
        target_x_relative = avg_x + 0.75 
        
        target.pose.position.x = current_pose.pose.position.x + target_x_relative
        target.pose.position.y = current_pose.pose.position.y + center_y
        target.pose.position.z = center_z
        target.pose.orientation = current_pose.pose.orientation
        
        gate_pose_target = target
        gate_world_x_position = current_pose.pose.position.x + avg_x

        rospy.loginfo("计算出门框目标点（世界坐标）: x=%.2f, y=%.2f, z=%.2f", 
                      target.pose.position.x, target.pose.position.y, target.pose.position.z)
        rospy.loginfo("门框平面世界坐标X位置: %.2f", gate_world_x_position)
        
        task_state = STATE_FLY_THROUGH_GATE


# --- 主函数 ---
if __name__ == "__main__":
    rospy.init_node("offb_and_lidar_node_py")

    # --- 订阅/发布/服务...
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=local_position_cb)
    lidar_sub = rospy.Subscriber("/rflysim/sensor0/mid360_lidar", PointCloud2, callback=lidar_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_once("等待飞控连接...")
        rate.sleep()
    rospy.loginfo("飞控已连接")

    # ### <<< 核心修改点: 更新航点列表以实现折线飞行 ###
    # 假设门在 x=4 处
    waypoints = [
        [0, 0, 1],         # 1. 起飞到1米高
        [0, 1.8, 1],      # 2. 飞到墙边 (完成折线第一部分)
        [2.5, 1.8, 1]     # 3. 飞到门前1.5米处 (完成折线第二部分), 然后开始搜索
    ]
    # --------------------------------------------------------
    
    waypoint_index = 0
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = waypoints[0]
    for i in range(100):
        if rospy.is_shutdown(): break
        local_pos_pub.publish(pose)
        rate.sleep()
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()

    # --- 主循环 (完全保持不变) ---
    while not rospy.is_shutdown():
        # 解锁和模式切换...
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD 模式已启用")
            last_req = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("无人机已解锁")
            last_req = rospy.Time.now()
        
        # 状态机逻辑...
        if task_state == STATE_WAYPOINTS:
            # 这段逻辑会自动遍历所有航点
            current_target_waypoint = waypoints[waypoint_index]
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = current_target_waypoint
            dist = math.sqrt(
                (current_pose.pose.position.x - current_target_waypoint[0])**2 +
                (current_pose.pose.position.y - current_target_waypoint[1])**2 +
                (current_pose.pose.position.z - current_target_waypoint[2])**2
            )
            if dist < 0.2: # 到达航点阈值可以根据需要调整
                rospy.loginfo("已到达航点 %d: %s", waypoint_index, str(current_target_waypoint))
                # 检查是否还有下一个航点
                if waypoint_index < len(waypoints) - 1:
                    waypoint_index += 1
                else:
                    # 所有航点都飞完了，进入搜索状态
                    rospy.loginfo("航点任务完成，开始搜索门框...")
                    task_state = STATE_SEARCH_GATE
        elif task_state == STATE_SEARCH_GATE:
            # 悬停，等待雷达回调
            pass
        elif task_state == STATE_FLY_THROUGH_GATE:
            if gate_pose_target:
                pose = gate_pose_target
                if gate_world_x_position is not None and current_pose.pose.position.x > gate_world_x_position:
                    rospy.loginfo("已穿过门框平面! 立即悬停。")
                    pose = current_pose 
                    task_state = STATE_DONE
            else:
                rospy.logwarn("门目标丢失，返回搜索状态")
                task_state = STATE_SEARCH_GATE
        elif task_state == STATE_DONE:
            rospy.loginfo_once("任务完成，悬停10秒...")
            start_hover_time = rospy.Time.now()
            while (rospy.Time.now() - start_hover_time).to_sec() < 10.0 and not rospy.is_shutdown():
                local_pos_pub.publish(pose)
                rate.sleep()
            rospy.loginfo("悬停结束，任务最终完成。")
            break

        local_pos_pub.publish(pose)
        rospy.loginfo_throttle(2.0, "当前任务状态: %d | 当前位置: x=%.2f, y=%.2f, z=%.2f",
                               task_state,
                               current_pose.pose.position.x,
                               current_pose.pose.position.y,
                               current_pose.pose.position.z)
        rate.sleep()

    rospy.loginfo("......................所有任务已完成..............")