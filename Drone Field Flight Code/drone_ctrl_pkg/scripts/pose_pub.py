#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped



"""
实现无人机的指点飞行
在键盘中输入无人机需要抵达的位置，无人机飞行至这个位置
在这里发布无人机飞行位置的信息
"""


def main():

    rospy.init_node('pos_pub')

    pos_pub=rospy.Publisher('pose_set_cmd',PoseStamped,queue_size=10)

    while not rospy.is_shutdown():

        rospy.loginfo("请输入无人机飞行的位置")
        pos_x=(input("请输入x="))
        if pos_x=='q':
            rospy.loginfo("----退出程序-----")
            break
        pos_x=float(pos_x)
        pos_y=float(input("请输入y="))
        pos_z=float(input("请输入z="))

        pos=PoseStamped()

        pos.pose.position.x=pos_x
        pos.pose.position.y=pos_y
        pos.pose.position.z=pos_z

        # 发布无人机需要飞行的位置
        pos_pub.publish(pos)

        rospy.loginfo("-----------------")
# rospy.spin()



if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass