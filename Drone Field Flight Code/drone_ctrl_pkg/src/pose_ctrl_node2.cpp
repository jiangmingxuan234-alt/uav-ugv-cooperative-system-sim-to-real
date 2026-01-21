#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64MultiArray.h>

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::State current_state;

bool manual_override = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void alter_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    float dx = msg->data[0];
    float dy = msg->data[1];
    float dz = msg->data[2];
    pose.pose.position.x += dx;
    pose.pose.position.y += dy;
    pose.pose.position.z += dz;
    manual_override = true;
    ROS_INFO("Manual adjustment: dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz);
}

void set_pose_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    pose.pose.position.x = msg->data[0];
    pose.pose.position.y = msg->data[1];
    pose.pose.position.z = msg->data[2];
    manual_override = true;
    ROS_INFO("Manual set position: x=%.2f, y=%.2f, z=%.2f", msg->data[0], msg->data[1], msg->data[2]);
}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void move_by(ros::Publisher& pub, ros::Rate& rate, float dx, float dy, float dz, int steps = 100){
    float step_dx = dx / steps;
    float step_dy = dy / steps;
    float step_dz = dz / steps;

    for(int i = 0; i < steps && ros::ok(); ++i){
        if (manual_override) return;
        pose.pose.position.x += step_dx;
        pose.pose.position.y += step_dy;
        pose.pose.position.z += step_dz;
        pose.header.stamp = ros::Time::now();
        pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 0; i < 40 && ros::ok(); ++i){
        if (manual_override) return;
        pose.header.stamp = ros::Time::now();
        pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void descend_to(ros::Publisher& pub, ros::Rate& rate, float target_z){
    float current_z = pose.pose.position.z;
    while(ros::ok() && current_z > target_z && !manual_override){
        current_z -= 0.05;
        pose.pose.position.z = current_z;
        for (int i = 0; i < 5; ++i){
            pose.header.stamp = ros::Time::now();
            pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_ctrl_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber alter_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>("alter_pose_cmd", 10, alter_pose_cb);
    ros::Subscriber set_pose_sub = nh.subscribe<std_msgs::Float64MultiArray>("set_pose_cmd", 10, set_pose_cb);
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, current_pose_cb);

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.0;

    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && (!current_state.armed || current_state.mode != "OFFBOARD")){
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD mode enabled");
            }
            last_request = ros::Time::now();
        } else if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Starting path mission...");

    move_by(local_pos_pub, rate,  0.0,  0.0,  0.0);  // 保持初始位置悬停
    move_by(local_pos_pub, rate,  0.0,  2.5,  0.0);  // 向右 2.5m
    move_by(local_pos_pub, rate,  4.0,  0.0,  0.0);  // 向前 4m
    move_by(local_pos_pub, rate,  0.0, -1.5,  0.0);  // 向左 1.5m
    move_by(local_pos_pub, rate, -2.0,  0.0,  0.0);  // 向后 2m
    move_by(local_pos_pub, rate,  0.0, -1.45, 0.0);  // 向左 1.45m

    descend_to(local_pos_pub, rate, 0.5);            // 平滑降落至 0.5m

    ROS_INFO("Path mission complete. Hovering...");

    // 悬停
    while(ros::ok()){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




