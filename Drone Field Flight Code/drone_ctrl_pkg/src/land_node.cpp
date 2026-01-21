#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>

void land()
{
    ros::NodeHandle nh;
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO.LAND";

    ROS_INFO("Waiting for services...");
    set_mode_client.waitForExistence();
    ROS_INFO("Services ready!");

    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
    {
        ROS_INFO("Landing mode sent.");
    }
    else
    {
        ROS_ERROR("Failed to send landing mode.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "land_node");
    land();
    return 0;
}