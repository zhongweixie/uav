#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <control_demo/public_tf.h>
#include <control_demo/image_srv.h>
// 定义pi
#define M_PI 3.14159265358979323846
using namespace std;
// 定义状态为枚举类型
enum STATE_ENUM
{
    start,
    task1,
    task2,
    stop
};
enum ACTION_ENUM
{
    noaction,
    action1,
    action2,
    action3,
    action4,
    action5,
    actiontimeout,
    // 此处actionend必须放最后
    actionend,
};
// 定义状态的管理类
class STATE
{
public:
    STATE()
    {
        for (int i = 0; i < actionend; i++)
        {
            actioncountMap[static_cast<ACTION_ENUM>(i)] = 0;
        }
    }
    STATE_ENUM pre_state = start;
    STATE_ENUM now_state = start;
    ACTION_ENUM now_action = noaction;
    bool flag_state_doonce = false;
    bool flag_state_actiondone = false;
    double state_start_time;
    std::map<ACTION_ENUM, int> actioncountMap;
    void state_start()
    {
        state_start_time = ros::Time::now().toSec();
        flag_state_doonce = true;
        flag_state_actiondone = false;
        now_action = noaction;
        for (int i = 0; i < actionend; i++)
        {
            actioncountMap[static_cast<ACTION_ENUM>(i)] = 0;
        }
    }

    double state_duration_time()
    {
        double state_duration_time = ros::Time::now().toSec() - state_start_time;
        return state_duration_time;
    }
    bool state_check_action(ACTION_ENUM action, double starttime, double endtime)
    {
        if (state_duration_time() >= starttime && state_duration_time() < endtime)
        {
            now_action = action;
            return true;
        }
        else
        {
            return false;
        }
    }
    void state_next(STATE_ENUM next)
    {
        pre_state = now_state;
        now_state = next;
        flag_state_doonce = false;
        flag_state_actiondone = false;
        now_action = noaction;
        for (int i = 0; i < actionend; i++)
        {
            actioncountMap[static_cast<ACTION_ENUM>(i)] = 0;
        }
    }
};

// 定义发布移动的类
class MOVE
{
public:
    MOVE(ros::NodeHandle &nh, PUBLIC_TF &public_tf, STATE &outstate, tf2_ros::Buffer &tfBuffer) : instate(outstate), TFBuffer(tfBuffer), Public_tf(public_tf)
    {
        tf2_ros::TransformListener tfListener(tfBuffer);
        now_odom_sub = nh.subscribe("turtle1/odom", 1, &MOVE::now_odomCallback, this);
        find_obj_sub = nh.subscribe("turtle1/find_obj", 1, &MOVE::find_objCallback, this);
        target_odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("target_position_odom", 10);
        image_req = nh.serviceClient<control_demo::image_srv>("image_server");
        maptoodom.child_frame_id = "odom";
        maptoodom.header.frame_id = "map";
        maptoodom.transform.translation.x = 0.0;
        maptoodom.transform.translation.y = 0.0;
        maptoodom.transform.translation.z = 0.0;
        maptoodom.transform.rotation.w = 1.0;
        maptoodom.transform.rotation.x = 0.0;
        maptoodom.transform.rotation.y = 0.0;
        maptoodom.transform.rotation.z = 0.0;
        Public_tf.pub_static(maptoodom);
        targetpoint_realinmap.header.frame_id = "map";
    }
    ros::Subscriber now_odom_sub;
    ros::Publisher target_odom_pub;
    ros::Subscriber find_obj_sub;
    ros::ServiceClient image_req;
    double error_dis = 0.1;
    double error_angle = 5.0;
    tf2::Quaternion myQuaternion;
    geometry_msgs::PoseWithCovarianceStamped robot_now_position;
    geometry_msgs::PoseWithCovarianceStamped robot_target_position;
    geometry_msgs::TransformStamped maptoodom;
    geometry_msgs::TransformStamped maptomapx;
    geometry_msgs::TransformStamped camera_linktotarget;
    geometry_msgs::PointStamped movepointinmapx;
    geometry_msgs::PointStamped movepointinmap;
    geometry_msgs::PointStamped movepoinindom;
    geometry_msgs::PointStamped targetpoint_realinmap;
    geometry_msgs::PointStamped targetpoint_inodom;
    geometry_msgs::PointStamped targetpoint_incamera_link;
    bool flag_mod_odominmap;
    STATE instate;
    tf2_ros::Buffer &TFBuffer;
    PUBLIC_TF &Public_tf;

    void find_obj(string detector,string det_object, bool mod_odominmap, double target_realinmap_x, double target_realinmap_y, double target_realinmap_z)
    {
        targetpoint_realinmap.point.x = target_realinmap_x;
        targetpoint_realinmap.point.y = target_realinmap_y;
        targetpoint_realinmap.point.z = target_realinmap_z;

        flag_mod_odominmap = mod_odominmap;
        control_demo::image_srv req_cmd;
        req_cmd.request.req_detector = detector;
        req_cmd.request.req_object = det_object;
        req_cmd.request.munber = 0;
        bool result = image_req.call(req_cmd);
        if (result)
        {
            ROS_INFO("服务申请成功");
            ROS_INFO("服务端执行结果为：%d-%s", req_cmd.response.success, req_cmd.response.msg.c_str());
        }
        else
        {
            ROS_INFO("服务申请失败");
        }
    }
    // 发送移动命令
    void send_moveto(double x, double y, double yaw, string mapx, bool mod_odominmap)
    {
        flag_mod_odominmap = mod_odominmap;
        // 把其它mapX下的坐标转换为map系下的坐标
        movepointinmapx.header.frame_id = mapx;
        movepointinmapx.point.x = x;
        movepointinmapx.point.y = y;
        movepointinmapx.point.z = 0.0;
        try
        {

            movepointinmap = TFBuffer.transform(movepointinmapx, "map");
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        ROS_INFO("Point in map frame: (%.2f, %.2f, %.2f)", movepointinmap.point.x, movepointinmap.point.y, movepointinmap.point.z);
        // 把map系下的坐标转换为odom系下的坐标

        try
        {
            movepoinindom = TFBuffer.transform(movepointinmap, "odom");
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        ROS_INFO("Point in odom frame: (%.2f, %.2f, %.2f)", movepoinindom.point.x, movepoinindom.point.y, movepoinindom.point.z);
        robot_target_position.pose.pose.position.x = movepoinindom.point.x;
        robot_target_position.pose.pose.position.y = movepoinindom.point.y;
        robot_target_position.pose.pose.position.z = 0.0;
        double rad_yaw = yaw * M_PI / 180.0;
        myQuaternion.setRPY(0, 0, rad_yaw);
        robot_target_position.pose.pose.orientation.w = myQuaternion.getW();
        robot_target_position.pose.pose.orientation.x = myQuaternion.getX();
        robot_target_position.pose.pose.orientation.y = myQuaternion.getY();
        robot_target_position.pose.pose.orientation.z = myQuaternion.getZ();
        target_odom_pub.publish(robot_target_position);
    }
    // 判断移动是否完成
    bool check_moveto()
    {
        double delta_x = robot_target_position.pose.pose.position.x - robot_now_position.pose.pose.position.x;

        double delta_y = robot_target_position.pose.pose.position.y - robot_now_position.pose.pose.position.y;

        double dis = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

        double delta_yaw = abs(tf::getYaw(robot_target_position.pose.pose.orientation) - tf::getYaw(robot_now_position.pose.pose.orientation)) * 180.0 / M_PI;
        if (delta_yaw < error_angle && dis < error_dis)
        {
            ROS_INFO("到达目标点:距离%lf角度%lf", dis, delta_yaw);
            return true;
        }
        else
        {
            ROS_INFO("未到:距离%lf角度%lf", dis, delta_yaw);
            return false;
        }
    }
    void now_odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    {
        robot_now_position = *msg;
    }

    void find_objCallback(const geometry_msgs::TransformStampedConstPtr &msg)
    {
        // 对于每个action中的find到的物体数量计数
        instate.actioncountMap[instate.now_action] = instate.actioncountMap[instate.now_action] + 1;
        // 修正map与odom间的关系
        if (flag_mod_odominmap)
        {
            camera_linktotarget = *msg;
            targetpoint_incamera_link.header.frame_id = "camera_link";
            targetpoint_incamera_link.point.x = camera_linktotarget.transform.translation.x;
            targetpoint_incamera_link.point.y = camera_linktotarget.transform.translation.y;
            targetpoint_incamera_link.point.z = camera_linktotarget.transform.translation.z;
            try
            {

                targetpoint_inodom = TFBuffer.transform(targetpoint_incamera_link, "odom");
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }
            ROS_INFO("Point in map frame: (%.2f, %.2f, %.2f)", targetpoint_inodom.point.x, targetpoint_inodom.point.y, targetpoint_inodom.point.z);
            // 计算targetpoint在map与odom系之间的误差,并发布；
            maptoodom.transform.translation.x = targetpoint_realinmap.point.x - targetpoint_inodom.point.x;
            maptoodom.transform.translation.y = targetpoint_realinmap.point.y - targetpoint_inodom.point.y;
            maptoodom.transform.translation.z = targetpoint_realinmap.point.z - targetpoint_inodom.point.z;
            Public_tf.pub_static(maptoodom);
        }
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "maincontrol");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;

    STATE state;
    PUBLIC_TF public_tf;
    MOVE move(nh, public_tf, state, tfBuffer);
    // ros::service::waitForService("cmd_service");
    ros::Duration(5).sleep();
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        switch (state.now_state)
        {
        case start:
        {
            // 判断单次任务是否执行
            if (state.flag_state_doonce == false)
            {
                state.state_start();
                // 下面为开始执行一次的任务
                ros::Duration(3).sleep();
            }

            state.state_next(task1);
            break;
        }
        case task1:
        {
            // 判断单次任务是否执行
            if (state.flag_state_doonce == false)
            {
                state.state_start();
                // 下面为状态开始时执行一次的任务

                move.find_obj("yolov5","obj_target1", true, 2.0, 0.0, 0.0);
                ros::Duration(1).sleep();
            }
            // 下面为状态持续时反复执行的任务包括action1，action2等
            if (state.state_check_action(action1, 0.0, 5))
            {
                move.send_moveto(-0.5, 0.0, 0.0, "map1", true);

                if (move.check_moveto() && state.actioncountMap[state.now_action] >= 0)
                {

                    state.flag_state_actiondone = true;
                }
            }
            else if (state.state_check_action(action2, 5, 10))
            {
                move.send_moveto(-1, 0.0, 0.0, "map1", true);
                if (move.check_moveto() && state.actioncountMap[state.now_action] >= 0)
                {
                    state.flag_state_actiondone = true;
                }
            }
            else
            {
                state.now_action = actiontimeout;
            }
            if (state.flag_state_actiondone || state.now_action == actiontimeout)
            {
                move.find_obj("nothing","nothing", false, 0.0, 0.0, 0.0);
                ros::Duration(1).sleep();
                state.state_next(task2);
                // 下面为状态结束时执行一次的任务
            }
            break;
        }
        case task2:
        {
            // 判断单次任务是否执行
            if (state.flag_state_doonce == false)
            {
                state.state_start();
                // 下面为状态开始时执行一次的任务

                move.find_obj("yolov5","all", true, 2.0, 0.0, 0.0);
                ros::Duration(1).sleep();
            }
            // 下面为状态持续时反复执行的任务包括action1，action2等
            if (state.state_check_action(action1, 0.0, 20))
            {
                move.send_moveto(-0.5, 0.0, 0.0, "map2", true);

                if (move.check_moveto() && state.actioncountMap[state.now_action] >= 0)
                {

                    state.flag_state_actiondone = true;
                }
            }
            else if (state.state_check_action(action2, 20, 40))
            {
                move.send_moveto(-1, 0.0, 0.0, "map2", true);
                if (move.check_moveto() && state.actioncountMap[state.now_action] >= 0)
                {
                    state.flag_state_actiondone = true;
                }
            }
            else
            {
                state.now_action = actiontimeout;
            }
            if (state.flag_state_actiondone || state.now_action == actiontimeout)
            {
                move.find_obj("nothing", "nothing", false, 0.0, 0.0, 0.0);
                ros::Duration(1).sleep();
                state.state_next(stop);
                // 下面为状态结束时执行一次的任务
            }
            break;
        }
        case stop:
        {
            // 判断单次任务是否执行
            if (state.flag_state_doonce == false)
            {
                state.state_start();
                // 下面为状态开始时执行一次的任务
                ros::Duration(5).sleep();
            }
            // 下面为状态持续时反复执行的任务
            ros::Duration(5).sleep();
            break;
        }
        default:
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
