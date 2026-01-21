//跟踪红色气球
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

geometry_msgs::Point current_balloon_position;
bool balloon_detected = false;

void balloon_position_cb(const geometry_msgs::Point::ConstPtr& msg) {
    current_balloon_position = *msg;
    balloon_detected = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "red_balloon_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber balloon_sub = nh.subscribe<geometry_msgs::Point>("/ballon_position", 10, balloon_position_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 确保与飞行器连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化速度指令
    geometry_msgs::TwistStamped current_vel_cmd;
    current_vel_cmd.header.frame_id = "map";
    current_vel_cmd.twist.linear.x = 0.0;
    current_vel_cmd.twist.linear.y = 0.0;
    current_vel_cmd.twist.linear.z = 0.0;
    current_vel_cmd.twist.angular.x = 0.0;
    current_vel_cmd.twist.angular.y = 0.0;
    current_vel_cmd.twist.angular.z = 0.0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        if (balloon_detected) {
            // 根据气球位置计算速度指令
            double error_x = current_balloon_position.x - 320.0; // 假设图像宽度为640
            double error_y = current_balloon_position.y - 240.0; // 假设图像高度为480
            double error_z = 100.0; // 假设目标距离为1米

            double speed_x = 0.0;
            double speed_z = 0.0;
            double speed_yaw = 0.0;

            if (fabs(error_x) > 10) {
                speed_yaw = -error_x * 0.005; // 调整偏航角速度
            }

            if(fabs(error_y)>10)
            {
                speed_z =-error_y*0.005;//调整高度
            }

            if (fabs(error_z) > 0.1) {
                speed_x = 0; // 调整前进速度
            }

            // 更新速度指令
            current_vel_cmd.header.stamp = ros::Time::now();
            current_vel_cmd.twist.linear.x = speed_x;
            current_vel_cmd.twist.linear.z = speed_z;
            current_vel_cmd.twist.angular.z = speed_yaw;

            for(int i=1;i<100;i++){
            local_vel_pub.publish(current_vel_cmd);
            }
        }

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

        local_vel_pub.publish(current_vel_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}