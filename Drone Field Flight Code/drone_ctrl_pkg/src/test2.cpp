#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>

void pose_cb( const std_msgs::Float64MultiArray::ConstPtr& msg){
    float dx,dy,dz;
    dx = msg->data[0];
    dy = msg->data[1];    
    dz = msg->data[2];
    ROS_INFO("Received pose adjustment: dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test2_node");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe<std_msgs::Float64MultiArray>("alter_pose_cmd", 10, pose_cb);


    ros::spinOnce();
    return 0;

}