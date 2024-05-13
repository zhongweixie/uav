#include "controller/fsm.h"

void fsm::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
{
    nh_ = nh;
    nh_private_ = nh_private;

    

    taking_off_pose_.pose.position.x = 0;
    taking_off_pose_.pose.position.y = 0;
    taking_off_pose_.pose.position.z = 0.4;
    taking_off_pose_.pose.orientation.x = 0;
    taking_off_pose_.pose.orientation.y = 0;
    taking_off_pose_.pose.orientation.z = 0;
    taking_off_pose_.pose.orientation.w = 1;
    takeoff_pos_ = Eigen::Vector3d(taking_off_pose_.pose.position.x, taking_off_pose_.pose.position.y, taking_off_pose_.pose.position.z);


    /*--------------------subscribe---------------------*/
    ego_pos_cmd_ = nh_.subscribe<quadrotor_msgs::PositionCommand>
                        ("planning/pos_cmd",10, boost::bind(&fsm::quadmsgCallback, this, _1));
    state_sub_ = nh_.subscribe<mavros_msgs::State>
                        ("mavros/state", 10, boost::bind(&fsm::px4_state_Callback, this, _1));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>
                        ("mavros/local_position/odom", 10, boost::bind(&fsm::odomCallback, this, _1));
    /*--------------------Timer callback-----------------*/
    fsm_loop_timer_ = nh_.createTimer(ros::Duration(0.01), &fsm::fsm_state_loopCallback, this); // Define timer for constant loop rate
    /*---------------------service-------------------------*/
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    /*----------------------publish-----------------------------*/
    pos_track_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_track_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10); 

    /*-------------------------param--------------------------------*/
    nh_private_.param("use_sim_", use_sim_, false);
    int mode = 0;
    nh_private_.param("track_mode_", mode, 0);
    track_mode_ = static_cast<fsm::TRACK_MODE>(mode);

    fsm_state_ = INIT;

}

void fsm::px4_state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    px4_last_state_ = px4_current_state_;
    px4_current_state_ = *msg;
    if(px4_current_state_.mode != px4_last_state_.mode)
    {
         ROS_INFO("Change mavMode From %s To: %s", px4_last_state_.mode.c_str(), px4_current_state_.mode.c_str());
    }
}

void fsm::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom_pos_(0) = odomMsg->pose.pose.position.x;
    odom_pos_(1) = odomMsg->pose.pose.position.y;
    odom_pos_(2) = odomMsg->pose.pose.position.z;

    odom_vel_(0) = odomMsg->twist.twist.linear.x;
    odom_vel_(1) = odomMsg->twist.twist.linear.y;
    odom_vel_(2) = odomMsg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = odomMsg->pose.pose.orientation.w;
    odom_orient_.x() = odomMsg->pose.pose.orientation.x;
    odom_orient_.y() = odomMsg->pose.pose.orientation.y;
    odom_orient_.z() = odomMsg->pose.pose.orientation.z;

    have_odom_ = true;

}

void fsm::quadmsgCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
    targetPos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    targetVel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    targetAcc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    ego_Yaw_ = double(cmd->yaw);
    changeFSMState(TRACKING, "EGO_MSG");
}

void fsm::changeFSMState(FSM_STATE new_state, string pos_call)
{
    static string state_str[7] = {"INIT", "TAKING_OFF", "WAIT_TARGET", "HOLD", "EXEC_MISSION", "ACROSS", "TRACKING"};
    int pre_s = int(fsm_state_);
    fsm_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void fsm::printFSMState()
{
    static string state_str[7] = {"INIT", "TAKING_OFF", "WAIT_TARGET", "HOLD", "EXEC_MISSION", "ACROSS", "TRACKING"};
    cout << "[FSM]: state: " + state_str[int(fsm_state_)] << endl;
}

void fsm::fsm_state_loopCallback(const ros::TimerEvent &event)
{
    fsm_loop_timer_.stop();
    static bool have_takeoff = false;
    static ros::Time last_request = ros::Time::now();
    /*----------一秒打印一次fsm状态----------*/
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMState();
      if (!have_odom_)
        ROS_WARN("no odom!!");
      fsm_num = 0;
    }

    switch ((fsm_state_))
    {
    case INIT:
    {
        if(!px4_current_state_.connected)
        {
            ROS_WARN("DISCONNECTED!RETRY...");
            goto force_return;
        }
        else
        {
            ROS_INFO("CONNECTED!");
        }
        if(!have_odom_)
        {
            goto force_return;
        }

        changeFSMState(TAKING_OFF, "FSM");
        break;
    }
    
    case TAKING_OFF:
    {
        if(use_sim_)
        {
            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            if( px4_current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else 
            {
                if( !px4_current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( arming_client_.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        pos_track_pub_.publish(taking_off_pose_);
        if((odom_pos_ - takeoff_pos_).norm()<0.1)
        {            
            changeFSMState(HOLD, "FSM");
            hold_pose_ = taking_off_pose_;
        }      


        break;

    }
    case HOLD://保持在空中悬停，悬停位姿需要在别处设置，刚起飞时为taking_off_pose_，之后则为刚进入hold状态时的里程计位姿
    {
        // if(!have_takeoff)
        // {
        //     hold_pose_ = taking_off_pose_;
        //     have_takeoff = true;
        // }
        // else
        // {
        //     hold_pose_.pose.position.x = odom_pos_(0);
        //     hold_pose_.pose.position.y = odom_pos_(1);
        //     hold_pose_.pose.position.z = odom_pos_(2);
        //     hold_pose_.pose.orientation.x = odom_orient_.x();
        //     hold_pose_.pose.orientation.y = odom_orient_.y();
        //     hold_pose_.pose.orientation.z = odom_orient_.z();
        //     hold_pose_.pose.orientation.w = odom_orient_.w();
        // }
        // // pos_track_(hold_pose_);
        pos_track_pub_.publish(hold_pose_);
        break;
    }
        
    case EXEC_MISSION:
        /* code */
        break;
    case WAIT_TARGET:
        /* code */
        break;
    case ACROSS:
        /* code */
        break;
    case TRACKING:
    {
        static string track_str_[2] = {"POS_TRACK", "VEL_TRACK"};
        if(track_mode_ == POS_TRACK)
        {
            ROS_INFO("Tracking mode: %s, next pos: [%f, %f, %f]", track_str_[int(track_mode_)].c_str(), targetPos_(0), targetPos_(1), targetPos_(2));

            pos_track_();
        }
        else if(track_mode_ == VEL_TRACK)
        {
            ROS_INFO("Tracking mode: %s, next pos: [%f, %f, %f]", track_str_[int(track_mode_)].c_str(), targetPos_(0), targetPos_(1), targetPos_(2));
            vel_track_();
        }
        else
        {
            ROS_WARN("track_mode invalid!");
        }
        break;
    }
    default:
        break;
    }
    force_return:;
    fsm_loop_timer_.start();
}

void fsm::pos_track_()
{
    geometry_msgs::PoseStamped track_pos_;
    track_pos_.header.stamp = ros::Time::now();
    track_pos_.pose.position.x = targetPos_(0);
    track_pos_.pose.position.y = targetPos_(1);
    track_pos_.pose.position.z = targetPos_(2);
    track_pos_.pose.orientation.x = 0;
    track_pos_.pose.orientation.y = 0;
    track_pos_.pose.orientation.z = 0;
    track_pos_.pose.orientation.w = 1;
    pos_track_pub_.publish(track_pos_);
    // ROS_INFO("pos_track succeed!");
}

void fsm::vel_track_()
{
    mavros_msgs::PositionTarget current_goal;
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系
    current_goal.header.stamp = ros::Time::now();
    current_goal.type_mask = velocity_mask;//对应的掩码设置，mavros_msgs::PositionTarget消息格式
    current_goal.velocity.x =  (targetPos_(0) - odom_pos_(0))*1;
    current_goal.velocity.y =  (targetPos_(1) - odom_pos_(1))*1;
    current_goal.velocity.z =  (targetPos_(2) - odom_pos_(2))*1;
    current_goal.yaw = ego_Yaw_;
    ROS_INFO("Tracking Vel: [%f, %f, %f]", pow(current_goal.velocity.x, 2), pow(current_goal.velocity.y, 2), pow(current_goal.velocity.z, 2));
    vel_track_pub_.publish(current_goal);
    // ROS_INFO("pos_track succeed!");
}