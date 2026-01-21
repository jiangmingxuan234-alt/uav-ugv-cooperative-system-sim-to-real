#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/CommandBool.h>
#include <iostream>

bool balloon_detect = false;
geometry_msgs::Point current_ballon_pose;
std_msgs::Float64MultiArray pose_alter;

void callback(const geometry_msgs::Point::ConstPtr& msg) {
    balloon_detect = true;
    current_ballon_pose = *msg;
    ROS_INFO("Received point: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "balloon_position_subscriber");
    ros::NodeHandle nh;

    ros::Publisher alter_pose_pub = nh.advertise<std_msgs::Float64MultiArray>("/alter_pose_cmd", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Point>("/balloon_position", 10, callback);

    ros::Rate rate(20.0);

    pose_alter.data.clear();
    pose_alter.data.push_back(0);
    pose_alter.data.push_back(0);
    pose_alter.data.push_back(0);

    while (ros::ok()) {
        std::cout << "------" << std::endl;

        if (balloon_detect) {
            double error_x = current_ballon_pose.x - 200; // 图像宽度为400
            double error_y = current_ballon_pose.y - 200; // 图像高度为400

            if (error_x > 10) { 
                pose_alter.data[1] = -0.05;
            } else if (error_x < -10) {
                pose_alter.data[1] = 0.05;
            } else {
                pose_alter.data[1] = 0.0;
            }

            if (error_y > 10) {
                pose_alter.data[2] = -0.05;
            } else if (error_y < -10) {
                pose_alter.data[2] = 0.05;
            } else {
                pose_alter.data[2] = 0.0;
            }
            pose_alter.data[0]=0.05;
            ROS_INFO("dy= %.2f, dz= %.2f", pose_alter.data[1], pose_alter.data[2]);
            alter_pose_pub.publish(pose_alter);

            balloon_detect = false; // 重置标志，以便下次检测到气球时重新处理
        }

        ros::spinOnce(); // 处理消息队列中的新消息
        rate.sleep();    // 控制循环频率
    }

    return 0;
}