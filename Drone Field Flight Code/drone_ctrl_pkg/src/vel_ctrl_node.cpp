//接受 keyboard_input_node,发布速度指令
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float64MultiArray.h>


ros::Publisher local_vel_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Time last_request;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void key_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 更新速度指令
    geometry_msgs::TwistStamped current_vel_cmd;
    current_vel_cmd.header.frame_id = "map";
    current_vel_cmd.header.stamp = ros::Time::now();
    current_vel_cmd.twist.linear.x = msg->data[0];
    current_vel_cmd.twist.linear.y = msg->data[1];
    current_vel_cmd.twist.linear.z = msg->data[2];
    current_vel_cmd.twist.angular.z = msg->data[3];

    for(int i=0;i<100;i++){
    local_vel_pub.publish(current_vel_cmd);
    }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_control_node");
    
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber key_sub = nh.subscribe<std_msgs::Float64MultiArray>("keyboard_input", 10, key_cb);

    ros::Rate rate(20.0);

    // 确保与飞行器连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Velocity control node ready");

    last_request = ros::Time::now();

    while(ros::ok()) {
        // 处理模式切换和解锁
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
                last_request = ros::Time::now();
            }
        }

        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}