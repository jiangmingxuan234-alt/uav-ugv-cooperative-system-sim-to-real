#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#include <unistd.h>
#include <std_msgs/Float64MultiArray.h>

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_input_node");
    ros::NodeHandle nh;

    // 获取原始终端设置
    tcgetattr(STDIN_FILENO, &original_terminal_settings);
    set_conio_terminal_mode();
    atexit(reset_terminal_mode); // 程序退出时恢复终端设置

    ros::Publisher key_pub = nh.advertise<std_msgs::Float64MultiArray>("keyboard_input", 10);

    ros::Rate rate(20.0);

    std_msgs::Float64MultiArray key_msg;
    key_msg.data.resize(4); // x, y, z, yaw_rate

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

        // 更新消息
        key_msg.data[0] = vel_x;
        key_msg.data[1] = vel_y;
        key_msg.data[2] = vel_z;
        key_msg.data[3] = yaw_rate;

        // 发布消息
        key_pub.publish(key_msg);

        ros::spinOnce();
        rate.sleep();
    }

    reset_terminal_mode(); // 确保恢复终端设置
    return 0;
}