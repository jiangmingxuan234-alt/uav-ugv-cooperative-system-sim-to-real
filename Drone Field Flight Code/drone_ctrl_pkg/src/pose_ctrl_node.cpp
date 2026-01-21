#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

}

void alter_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    float dx,dy,dz;
    dx = msg->data[0];
    dy = msg->data[1];    
    dz = msg->data[2];
    pose.pose.position.x=pose.pose.position.x+dx;
    pose.pose.position.y =pose.pose.position.y+dy;
    pose.pose.position.z =pose.pose.position.z+dz;
    ROS_INFO("Received pose adjustment: dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz);
}

void set_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    pose.pose.position.x=msg->data[0];
    pose.pose.position.y=msg->data[1];
    pose.pose.position.z=msg->data[2];
    ROS_INFO("Moving pose to x=%.2f,y=%.2f,z=%.2f",msg->data[0],msg->data[1],msg->data[2]);

}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_ctrl_node");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber alter_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("alter_pose_cmd", 10, alter_pose_cb);
    ros::Subscriber set_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("set_pose_cmd", 10, set_pose_cb);
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10,current_pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}