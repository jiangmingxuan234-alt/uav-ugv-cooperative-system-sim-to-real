#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

void takeoff()
{
    ros::NodeHandle nh;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ROS_INFO("Waiting for services...");
    arming_client.waitForExistence();
    set_mode_client.waitForExistence();
    ROS_INFO("Services ready!");

    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
    {
        ROS_INFO("Takeoff mode sent.");
    }
    else
    {
        ROS_ERROR("Failed to send takeoff mode.");
    }

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed.");
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node");
    takeoff();
    return 0;
}