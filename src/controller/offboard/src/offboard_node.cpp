#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped pose;
void egoplanner_pose(quadrotor_msgs::PositionCommand pose_cmd)
{
    
    pose.pose.position.x = pose_cmd.position.x;
    pose.pose.position.y = pose_cmd.position.y;
    pose.pose.position.z = pose_cmd.position.z;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    //订阅mavros状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/iris_0/mavros/state", 10, state_cb);

    //订阅ego_planner发布的位姿消息
    ros::Subscriber pose_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
            ("/iris_0/planning/pos_cmd", 10, egoplanner_pose);

    //发布无人机位姿信息
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/iris_0/mavros/setpoint_position/local", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("/iris_0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    //定义起飞服务客户端（起飞，降落）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/iris_0/mavros/cmd/arming");
    //定义设置模式服务客户端（设置offboard模式）
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/iris_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!!!");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.4;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i)
    // {
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
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

