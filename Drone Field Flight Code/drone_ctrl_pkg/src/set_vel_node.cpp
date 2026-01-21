//使用方式为 rosrun... vx vy vz
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_vel_node");
    ros::NodeHandle nh;


	if (argc < 4)
    {
        ROS_ERROR("Usage: rosrun drone_ctrl_pkg set_point_node <x> <y> <z>");
        return -1;
    }

    double vx = atof(argv[1]);
    double vy = atof(argv[2]);
    double vz = atof(argv[3]);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.header.frame_id = "map";
    vel_cmd.header.stamp = ros::Time::now();

    vel_cmd.twist.linear.x = vx; // X方向速度
    vel_cmd.twist.linear.y = vy; // Y方向速度
    vel_cmd.twist.linear.z = vz; // Z方向速度

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(vel_cmd);
        ros::spinOnce();
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

        local_pos_pub.publish(vel_cmd);

        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}