#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <unistd.h> // 用于sleep函数



void move()
{
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 5.0;  // 目标位置
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 10.0;

    ros::Rate rate(20);  // 20 Hz

    // 先发布目标位置的初始值
    for (int i = 0; i < 100; ++i)
    {
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        
    }

    ROS_INFO("Moving drone to target position...");
    while (ros::ok())
    {
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_node");

    ros::NodeHandle nh;
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode set_mode;

    move();
    set_mode.request.custom_mode = "OFFBOARD";


    // 开始移动
    

    return 0;
}