#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#include <unistd.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

struct termios original_terminal_settings;

void reset_terminal_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_settings);
}

void set_conio_terminal_mode() {
    struct termios new_settings = original_terminal_settings;
    new_settings.c_lflag &= ~ICANON; // 禁用规范模式
    new_settings.c_lflag &= ~ECHO;   // 禁用回显
    new_settings.c_cc[VTIME] = 0;    // 定时器单位（十分之一秒）
    new_settings.c_cc[VMIN] = 1;     // 最少读取字符数
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

int getch() {
    int c = 0;
    c = getchar();
    return c;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_ctrl_node");
    ros::NodeHandle nh;

    // 获取原始终端设置
    tcgetattr(STDIN_FILENO, &original_terminal_settings);
    set_conio_terminal_mode();
    atexit(reset_terminal_mode); // 程序退出时恢复终端设置

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 初始化速度指令
    geometry_msgs::TwistStamped current_vel_cmd;
    current_vel_cmd.header.frame_id = "map";
    current_vel_cmd.twist.linear.x = 0.0;
    current_vel_cmd.twist.linear.y = 0.0;
    current_vel_cmd.twist.linear.z = 0.0;
    current_vel_cmd.twist.angular.x = 0.0;
    current_vel_cmd.twist.angular.y = 0.0;
    current_vel_cmd.twist.angular.z = 0.0;

    // 确保与飞行器连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化速度值
    double speed_step = 0.1; // 速度增量
    double max_speed = 5.0;  // 最大速度限制
    double vel_x = 0.0; // X方向速度
    double vel_y = 0.0; // Y方向速度
    double vel_z = 0.0; // Z方向速度
    double yaw_rate = 0.0; // 偏航角速度

    ROS_INFO("Ready to receive keyboard commands:");
    ROS_INFO("Use arrow keys to move (Up: forward, Down: backward, Left: left, Right: right)");
    ROS_INFO("Use W/S to ascend/descend, A/D to yaw left/right");
    ROS_INFO("X to exit");

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        // 处理键盘输入
        int ch = getch();
        switch(ch) {
            case 27: // Escape序列开始，可能是箭头键
                ch = getchar(); // 读取 [
                if (ch == '[') {
                    ch = getchar(); // 读取箭头键代码
                    switch(ch) {
                        case 'A': // 向上箭头 - 向前
                            vel_x = std::min(vel_x + speed_step, max_speed);
                            break;
                        case 'B': // 向下箭头 - 向后
                            vel_x = std::max(vel_x - speed_step, -max_speed);
                            break;
                        case 'D': // 向左箭头 - 向左
                            vel_y = std::min(vel_y + speed_step, max_speed);
                            break;
                        case 'C': // 向右箭头 - 向右
                            vel_y = std::max(vel_y - speed_step, -max_speed);
                            break;
                    }
                }
                break;
            case 'w': // 上升
            case 'W':
                vel_z = std::min(vel_z + speed_step, max_speed);
                break;
            case 's': // 下降
            case 'S':
                vel_z = std::max(vel_z - speed_step, -max_speed);
                break;
            case 'a': // 左转
            case 'A':
                yaw_rate = std::min(yaw_rate + speed_step, max_speed);
                break;
            case 'd': // 右转
            case 'D':
                yaw_rate = std::max(yaw_rate - speed_step, -max_speed);
                break;
            case 'x': // 退出
            case 'X':
                reset_terminal_mode(); // 恢复终端设置
                return 0;
        }

        // 更新速度指令
        current_vel_cmd.header.stamp = ros::Time::now();
        current_vel_cmd.twist.linear.x = vel_x;
        current_vel_cmd.twist.linear.y = vel_y;
        current_vel_cmd.twist.linear.z = vel_z;
        current_vel_cmd.twist.angular.z = yaw_rate;

        // 发布速度指令
        
        for(int i=0;i<100;i++){
            local_vel_pub.publish(current_vel_cmd);
            ros::spinOnce();
            rate.sleep();
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

    reset_terminal_mode(); // 确保恢复终端设置
    return 0;
}