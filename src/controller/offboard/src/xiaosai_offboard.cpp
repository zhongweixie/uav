#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" //用来获取仿真环境中的当前位姿
#include "geometry_msgs/Pose.h" //用来获取目标位姿
#include "mavros_msgs/State.h" //无人机当前状态：连接状态、解锁状态、模式
#include "mavros_msgs/CommandBool.h" //控制无人机解锁
#include "mavros_msgs/SetMode.h" //控制无人机模式
#include "offboard/xiaosai_detect_type.h"

mavros_msgs::State current_state; //用来获取当前状态
geometry_msgs::PoseStamped current_position; //用来获取当前位置
geometry_msgs::Pose target_position; //用来获取目标位置
geometry_msgs::PoseStamped pose; //用来存储现在正在执行的目标位置

void state_cb(const mavros_msgs::State::ConstPtr& msg) //回调函数，处理状态消息
{
    current_state = *msg;
}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) //回调函数，处理当前位置消息
{
    current_position = *msg;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"xiaosai_offb_node_cpp"); //定义节点
    ros::NodeHandle nh; //定义句柄

    //参数服务器
    int is_detect = 0;
    double dx = 0.0;
    double dy = 0.0;
    nh.setParam("is_detect",0);
    nh.setParam("dx",0.0);
    nh.setParam("dy",0.0);
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, state_cb); //订阅状态信息
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, current_pose_cb); //订阅仿真环境中当前位置信息
    // ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::Pose>("/ego_pose_cmd", 10, target_pose_cb); //订阅目标位置信息
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/iris_0/mavros/setpoint_position/local", 10); //用来发布位置信息

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming"); //解锁服务
    ros::service::waitForService("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode"); //模式服务
    ros::service::waitForService("/iris_0/mavros/set_mode");

//发布目标点给ego_planner
    // ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    ros::Rate rate(20.0); // 以20Hz的频率发布数据，1秒20次

    while (ros::ok() && !current_state.connected) { //等待飞行器连接
        ros::spinOnce();
        rate.sleep();
    }

    //高度设置
    float detect_height = 1;
    float put_height = 0.1;
    float pass_height = 0.7;

    //无人机飞到（0，0，2）
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    // Send a few setpoints before starting
    for (int i = 0; i < 100 && ros::ok(); i++) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ROS_INFO("start position control!");
    ROS_INFO("第一次起飞！");

    ros::Time last_request = ros::Time::now();
    // ros::Time during_request = ros::Time::now();
    bool F_task_over = false;

    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        // if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        // {
        //     during_request = ros::Time::now();
        //     while (!F_task_over)
        //     {
        //         if ((ros::Time::now() - during_request) < ros::Duration(2.0))
        //         {
        //             local_pos_pub.publish(pose);
        //             ros::spinOnce();
        //         }
        //         else
        //         {
        //             F_task_over = true;
        //         }
        //     }
        // }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");

    //飞到（3.5，0，2），第一个投放点
    pose.pose.position.x = 3.0;
    pose.pose.position.y = 0;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    ROS_INFO("去往第一个投放点！");
    // goal_pub.publish(pose);//发布第一个投放点位置给ego

    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");

    ROS_INFO("请求识别目标");
    bool detect_over = false;
    // 创建客户端
    ros::ServiceClient client = nh.serviceClient<offboard::xiaosai_detect_type>("xiaosai_detect_task"); //解锁服务
    // 组织请求数据
    offboard::xiaosai_detect_type srv;
    srv.request.type = "detect";
    srv.request.num = 1;
    while(!detect_over){
        // detect_over = true;
        client.call(srv);
        ros::spinOnce();
        rate.sleep();
        // 显示响应
        ROS_INFO("%s", srv.response.is_detect.c_str());
        while(!is_detect)
        {
            local_pos_pub.publish(pose);
            ROS_INFO("悬停");
            is_detect = nh.param("is_detect",0);
            dx = nh.param("dx",0.0);
            dy = nh.param("dy",0.0);
        }
        if(abs(dx)<0.2&&abs(dy)<0.2)
        {
            detect_over = true;
            ROS_INFO("目标一识别完毕");
            nh.setParam("is_detect",0);
            is_detect = nh.param("is_detect",0);
            break;
        }
        ROS_INFO("%d", is_detect);
        ROS_INFO("dx:%f", dx);
        ROS_INFO("dy:%f", dy);
        pose.pose.position.x -= dy;
        pose.pose.position.y -= dx;
        nh.setParam("is_detect",0);
        nh.setParam("dx",0.0);
        nh.setParam("dy",0.0);
        is_detect = nh.param("is_detect",0);
        dx = nh.param("dx",0.0);
        dy = nh.param("dy",0.0);
        // ros::spinOnce();
        // rate.sleep();
        F_task_over = false;
        while (ros::ok() && !F_task_over) {
            ROS_INFO("开始调整位置");
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            ROS_INFO("current:x=%f, y=%f, z=%f",current_position.pose.position.x,current_position.pose.position.y,current_position.pose.position.z); //输出当前坐标
            ROS_INFO("pose:x=%f, y=%f, z=%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z); //输出当前坐标
            if(abs(current_position.pose.position.x-pose.pose.position.x)<0.1&&abs(current_position.pose.position.y-pose.pose.position.y)<0.1&&abs(current_position.pose.position.z-pose.pose.position.z)<0.1)
            {
                F_task_over = true;
                ROS_INFO("调整结束");
            }
        }
        // if(abs(dx)<0.01&&abs(dy)<0.01)
        // {
        //     detect_over = true;
        //     ROS_INFO("目标一识别完毕");
        // }
    }
    // ROS_INFO("%d", is_detect);
    // ROS_INFO("dx:%f", dx);
    // ROS_INFO("dy:%f", dy);
    

    ROS_INFO("开始投放");
    pose.pose.position.z = put_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("投放完成");
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("去往第二个投放点！");
    pose.pose.position.x = 3.3;
    pose.pose.position.y = -2;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    // goal_pub.publish(pose);//发布第二个投放点位置给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    pose.pose.position.x = 4.5;
    pose.pose.position.y = -2;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("请求识别目标");
    detect_over = false;
    srv.request.type = "detect";
    srv.request.num = 2;
    while(!detect_over){
        // detect_over = true;
        client.call(srv);
        ros::spinOnce();
        rate.sleep();
        // 显示响应
        ROS_INFO("%s", srv.response.is_detect.c_str());
        while(!is_detect)
        {
            local_pos_pub.publish(pose);
            ROS_INFO("悬停");
            is_detect = nh.param("is_detect",0);
            dx = nh.param("dx",0.0);
            dy = nh.param("dy",0.0);
        }
        if(abs(dx)<0.2&&abs(dy)<0.2)
        {
            detect_over = true;
            ROS_INFO("目标二识别完毕");
            nh.setParam("is_detect",0);
            is_detect = nh.param("is_detect",0);
            break;
        }
        ROS_INFO("%d", is_detect);
        ROS_INFO("dx:%f", dx);
        ROS_INFO("dy:%f", dy);
        pose.pose.position.x -= dy;
        pose.pose.position.y -= dx;
        nh.setParam("is_detect",0);
        nh.setParam("dx",0.0);
        nh.setParam("dy",0.0);
        is_detect = nh.param("is_detect",0);
        dx = nh.param("dx",0.0);
        dy = nh.param("dy",0.0);
        // ros::spinOnce();
        // rate.sleep();
        F_task_over = false;
        while (ros::ok() && !F_task_over) {
            ROS_INFO("开始调整位置");
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            ROS_INFO("current:x=%f, y=%f, z=%f",current_position.pose.position.x,current_position.pose.position.y,current_position.pose.position.z); //输出当前坐标
            ROS_INFO("pose:x=%f, y=%f, z=%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z); //输出当前坐标
            if(abs(current_position.pose.position.x-pose.pose.position.x)<0.1&&abs(current_position.pose.position.y-pose.pose.position.y)<0.1&&abs(current_position.pose.position.z-pose.pose.position.z)<0.1)
            {
                F_task_over = true;
                ROS_INFO("调整结束");
            }
        }
        // if(abs(dx)<0.01&&abs(dy)<0.01)
        // {
        //     detect_over = true;
        //     ROS_INFO("目标一识别完毕");
        // }
    }


    ROS_INFO("开始投放");
    pose.pose.position.z = put_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("投放完成");
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("去往第三个投放点！");
    pose.pose.position.x = 5;
    pose.pose.position.y = -1.5;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    // goal_pub.publish(pose);//发布第三个投放点位置给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    pose.pose.position.x = 5.2;
    pose.pose.position.y = -0.5;
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("请求识别目标");
    detect_over = false;
    srv.request.type = "detect";
    srv.request.num = 3;
    while(!detect_over){
        // detect_over = true;
        client.call(srv);
        ros::spinOnce();
        rate.sleep();
        // 显示响应
        ROS_INFO("%s", srv.response.is_detect.c_str());
        while(!is_detect)
        {
            local_pos_pub.publish(pose);
            ROS_INFO("悬停");
            is_detect = nh.param("is_detect",0);
            dx = nh.param("dx",0.0);
            dy = nh.param("dy",0.0);
        }
        if(abs(dx)<0.2&&abs(dy)<0.2)
        {
            detect_over = true;
            ROS_INFO("目标三识别完毕");
            nh.setParam("is_detect",0);
            is_detect = nh.param("is_detect",0);
            break;
        }
        ROS_INFO("%d", is_detect);
        ROS_INFO("dx:%f", dx);
        ROS_INFO("dy:%f", dy);
        pose.pose.position.x -= dy;
        pose.pose.position.y -= dx;
        nh.setParam("is_detect",0);
        nh.setParam("dx",0.0);
        nh.setParam("dy",0.0);
        is_detect = nh.param("is_detect",0);
        dx = nh.param("dx",0.0);
        dy = nh.param("dy",0.0);
        // ros::spinOnce();
        // rate.sleep();
        F_task_over = false;
        while (ros::ok() && !F_task_over) {
            ROS_INFO("开始调整位置");
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            ROS_INFO("current:x=%f, y=%f, z=%f",current_position.pose.position.x,current_position.pose.position.y,current_position.pose.position.z); //输出当前坐标
            ROS_INFO("pose:x=%f, y=%f, z=%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z); //输出当前坐标
            if(abs(current_position.pose.position.x-pose.pose.position.x)<0.1&&abs(current_position.pose.position.y-pose.pose.position.y)<0.1&&abs(current_position.pose.position.z-pose.pose.position.z)<0.1)
            {
                F_task_over = true;
                ROS_INFO("调整结束");
            }
        }
        // if(abs(dx)<0.01&&abs(dy)<0.01)
        // {
        //     detect_over = true;
        //     ROS_INFO("目标一识别完毕");
        // }
    }


    ROS_INFO("开始投放");
    pose.pose.position.z = put_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("投放完成");
    pose.pose.position.z = detect_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.3&&abs(current_position.pose.position.y-pose.pose.position.y)<0.3&&abs(current_position.pose.position.z-pose.pose.position.z)<0.3)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("穿越障碍");
    pose.pose.position.x = 6.5;
    pose.pose.position.y = 0.75;
    pose.pose.position.z = pass_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    // goal_pub.publish(pose);//发布开始穿越障碍物位置给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.05&&abs(current_position.pose.position.y-pose.pose.position.y)<0.05&&abs(current_position.pose.position.z-pose.pose.position.z)<0.05)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");

    pose.pose.position.x = 7;
    pose.pose.position.y = 0.75;
    pose.pose.position.z = pass_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    // goal_pub.publish(pose);//发布障碍位置2给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.2&&abs(current_position.pose.position.y-pose.pose.position.y)<0.2&&abs(current_position.pose.position.z-pose.pose.position.z)<0.2)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");

    pose.pose.position.x = 8.4;
    pose.pose.position.y = 0.75;
    pose.pose.position.z = pass_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    //  goal_pub.publish(pose);//发布障碍位置3给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.2&&abs(current_position.pose.position.y-pose.pose.position.y)<0.2&&abs(current_position.pose.position.z-pose.pose.position.z)<0.2)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");

    pose.pose.position.x = 8.4;
    pose.pose.position.y = -3.3;
    pose.pose.position.z = pass_height;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    //  goal_pub.publish(pose);//发布障碍位置4给ego
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request) > ros::Duration(5.0)) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        if(abs(current_position.pose.position.x-pose.pose.position.x)<0.2&&abs(current_position.pose.position.y-pose.pose.position.y)<0.2&&abs(current_position.pose.position.z-pose.pose.position.z)<0.2)
        {
            F_task_over = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\n");


    ROS_INFO("请求识别降落点");
    detect_over = false;
    srv.request.type = "land";
    srv.request.num = 4;
    while(!detect_over){
        // detect_over = true;
        client.call(srv);
        ros::spinOnce();
        rate.sleep();
        // 显示响应
        ROS_INFO("%s", srv.response.is_detect.c_str());
        while(!is_detect)
        {
            local_pos_pub.publish(pose);
            ROS_INFO("悬停");
            is_detect = nh.param("is_detect",0);
            dx = nh.param("dx",0.0);
            dy = nh.param("dy",0.0);
        }
        if(abs(dx)<0.1&&abs(dy)<0.1)
        {
            detect_over = true;
            ROS_INFO("降落点识别完毕");
            nh.setParam("is_detect",0);
            is_detect = nh.param("is_detect",0);
            break;
        }
        ROS_INFO("%d", is_detect);
        ROS_INFO("dx:%f", dx);
        ROS_INFO("dy:%f", dy);
        pose.pose.position.x -= dy;
        pose.pose.position.y -= dx;
        nh.setParam("is_detect",0);
        nh.setParam("dx",0.0);
        nh.setParam("dy",0.0);
        is_detect = nh.param("is_detect",0);
        dx = nh.param("dx",0.0);
        dy = nh.param("dy",0.0);
        // ros::spinOnce();
        // rate.sleep();
        F_task_over = false;
        while (ros::ok() && !F_task_over) {
            ROS_INFO("开始调整位置");
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            ROS_INFO("current:x=%f, y=%f, z=%f",current_position.pose.position.x,current_position.pose.position.y,current_position.pose.position.z); //输出当前坐标
            ROS_INFO("pose:x=%f, y=%f, z=%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z); //输出当前坐标
            if(abs(current_position.pose.position.x-pose.pose.position.x)<0.1&&abs(current_position.pose.position.y-pose.pose.position.y)<0.1&&abs(current_position.pose.position.z-pose.pose.position.z)<0.1)
            {
                F_task_over = true;
                ROS_INFO("调整结束");
            }
        }
        // if(abs(dx)<0.01&&abs(dy)<0.01)
        // {
        //     detect_over = true;
        //     ROS_INFO("目标一识别完毕");
        // }
    }


    offb_set_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = false;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    ROS_INFO("Start landing");
    last_request = ros::Time::now();
    F_task_over = false;
    while (ros::ok() && !F_task_over) {
        if ((ros::Time::now() - last_request) > ros::Duration(5)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Landing!");
                F_task_over = true;
            }
        }
        if ((ros::Time::now() - last_request) < ros::Duration(5)) {
            local_pos_pub.publish(pose);
            ros::spinOnce();
        }
        rate.sleep();
    }



    return 0;
}
