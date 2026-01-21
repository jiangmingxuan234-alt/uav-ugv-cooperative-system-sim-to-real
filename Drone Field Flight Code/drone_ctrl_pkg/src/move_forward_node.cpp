#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>

void setFlightMode(const std::string& mode)
{
    ros::NodeHandle nh;
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = mode;

    ROS_INFO("Waiting for services...");
    set_mode_client.waitForExistence();
    ROS_INFO("Services ready!");

    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
    {
        ROS_INFO("Mode set to %s.", mode.c_str());
    }
    else
    {
        ROS_ERROR("Failed to set mode to %s.", mode.c_str());
    }
}

void armDrone()
{
    ros::NodeHandle nh;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ROS_INFO("Waiting for services...");
    arming_client.waitForExistence();
    ROS_INFO("Services ready!");

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed.");
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle.");
    }
}

void publishVelocity()
{
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.header.frame_id = "map";

    // 设置速度指令
    vel_cmd.twist.linear.x = 1.0; // X方向速度（单位：米/秒）
    vel_cmd.twist.linear.y = 0.0;
    vel_cmd.twist.linear.z = 0.0;
    vel_cmd.twist.angular.x = 0.0;
    vel_cmd.twist.angular.y = 0.0;
    vel_cmd.twist.angular.z = 0.0;

    ros::Rate rate(20); // 20 Hz
    while (ros::ok())
    {
        vel_cmd.header.stamp = ros::Time::now();
        vel_pub.publish(vel_cmd);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_ctrl_node");

    

    // 解锁电机
    armDrone();

    // 发布速度指令
    publishVelocity();

    return 0;
}