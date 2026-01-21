#! /usr/bin/python3
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool,SetMode, SetModeRequest, CommandBoolRequest


# 无人机起飞的高度，默认为2m
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

# 实例化切换OFFBOARD模式和ARM的客户端
offb_set_mode = SetModeRequest()
offb_set_mode.custom_mode = 'OFFBOARD'

arm_cmd = CommandBoolRequest()
arm_cmd.value = True   


current_state=State()
# 定义回调函数 状态回调函数和位置回调函数
def state_callback(msg):
    global current_state
    current_state=msg


# 回调函数 更新无人机的位置为键盘输入的位置
def pos_callback(msg, pose):
    pose.pose.position.x=msg.pose.position.x
    pose.pose.position.y=msg.pose.position.y
    pose.pose.position.z=msg.pose.position.z
    rospy.loginfo(pose)


def pose_control():

    # 初始化一个节点
    px4_control_node=rospy.init_node('px4_control_node2')
    # 实例化消息

    # 实例化订阅、发布、服务等
    state_sub=rospy.Subscriber('/mavros/state',State,state_callback)

    # 发布无人机的位置
    target_pos_pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)

    arming_client=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    set_mode_client=rospy.ServiceProxy('/mavros/set_mode',SetMode)
    # 无人机 切换模式、解锁、起飞

    rate = rospy.Rate(20)# 设置话题发布频率

    while(not rospy.is_shutdown() and not current_state.connected):
        print("----连接出错-----")
        rate.sleep()

    # 起飞前先发布一些消息
    for _ in range(100):
        if(rospy.is_shutdown()):
            break
        target_pos_pub.publish(pose)
        rate.sleep()

    last_req = rospy.Time.now()

    # 订阅位置的输入，并传入 将要发布至无人机的消息
    rospy.Subscriber("pos_input", PoseStamped, pos_callback, pose)

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        target_pos_pub.publish(pose)     
        rate.sleep()


# 运行主程序
if __name__ == "__main__":
    pose_control()