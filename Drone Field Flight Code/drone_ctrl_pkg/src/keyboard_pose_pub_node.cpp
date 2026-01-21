#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_pose_pub_node");
    ros::NodeHandle nh;
    ros::Publisher local_pose_pub = nh.advertise<std_msgs::Float64MultiArray>("set_pose_cmd", 10);
    std_msgs::Float64MultiArray set_pose;
    ros::Rate rate(20);

    // 确保数组大小为3
    set_pose.data.resize(3);

    while (ros::ok()) {
        ROS_INFO("please set pose x y z");
        float x, y, z;
        std::cin >> x >> y >> z;

        set_pose.data[0] = x;
        set_pose.data[1] = y;
        set_pose.data[2] = z;

        local_pose_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}