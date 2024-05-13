#ifndef FSM_H
#define FSM_H
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <string>
#include <std_msgs/Bool.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <quadrotor_msgs/PositionCommand.h>

#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE

using namespace std;


class fsm
{
private:

  /*飞机的当前状态*/
  enum FSM_STATE
  {
    INIT,
    TAKING_OFF,
    WAIT_TARGET,
    HOLD,
    EXEC_MISSION,
    ACROSS,
    TRACKING
  }fsm_state_;

  enum TRACK_MODE
  {
    POS_TRACK,
    VEL_TRACK
  }track_mode_;



/*----------------------------------------------ROS utils-------------------------------------------------*/
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber ego_pos_cmd_, state_sub_, odom_sub_;
  ros::Publisher pos_track_pub_, vel_track_pub_;
  ros::ServiceClient arming_client_, set_mode_client_;

  /*接收来自ego的quadrotor_msgs，轨迹消息*/
  void quadmsgCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);
  /*定时回调，检查fsm的状态*/
  void fsm_state_loopCallback(const ros::TimerEvent &event);
  /*无人机的状态回调*/
  void px4_state_Callback(const mavros_msgs::State::ConstPtr& msg);
  /*订阅来自mavros的融合后的odom，mavros/local_position/odom*/
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

/*-------------------------------------------fsm utils-------------------------------------------------*/
  /*改变fsm状态，并输出调用位置和当前状态*/
  void changeFSMState(FSM_STATE new_state, string pos_call);
  /*打印fsm状态*/
  void printFSMState();

/*-------------------------------------------track utils-------------------------------------------------*/
  void pos_track_();
  void vel_track_();

private:

  bool use_sim_;
  mavros_msgs::State px4_current_state_, px4_last_state_;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;

  ros::Timer fsm_loop_timer_;
  geometry_msgs::PoseStamped taking_off_pose_;
  geometry_msgs::PoseStamped hold_pose_;

  bool have_odom_;
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
  Eigen::Quaterniond odom_orient_;                  //里程计中的四元数
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_;//ego中的目标位置
  double ego_Yaw_;                                    //ego中的目标角度
  Eigen::Vector3d takeoff_pos_;   //起飞位置
  Eigen::Vector3d hold_pos_;      //在空中悬停的坐标
  Eigen::Quaterniond hold_orient_;//在空中悬停的姿态

  unsigned short velocity_mask = VELOCITY2D_CONTROL;


public:
    fsm(){};
    ~fsm(){};
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);




};


#endif //FSM_H