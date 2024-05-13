// 头文件部分
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <turtlesim/Pose.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// 初始化部分
turtlesim::Pose now_pose;

void get_speedCallback(const turtlesim::Pose &msg)
{
    now_pose = msg;
}

class calculate_odom
{
public:
    calculate_odom(turtlesim::Pose *now_pose):pose(now_pose)
    {
    }

     geometry_msgs::PoseWithCovarianceStamped update()
    {

        double now_time = ros::Time::now().toSec();
        del_time = now_time - pre_time;
      
        double del_x =pose->x-pre_x;
        double del_y=pose->y-pre_y;
        std::cout<< "pose->y"<< pose->y <<std::endl;
        std::cout<< "pre_y"<< pre_y  <<std::endl;
          std::cout<< "del_y"<< del_y  <<std::endl;
        tf::Quaternion quat = tf::createQuaternionFromRPY(0 ,0, pose->theta);
       odom_position.pose.pose.orientation.x=quat.getX();
       odom_position.pose.pose.orientation.y=quat.getY();
       odom_position.pose.pose.orientation.z=quat.getZ();
       odom_position.pose.pose.orientation.w=quat.getW();
       odom_position.pose.pose.position.x=odom_position.pose.pose.position.x+del_x+0.00001 ;
       odom_position.pose.pose.position.y=odom_position.pose.pose.position.y+del_y+0.00001;
       pre_x =pose->x;
       pre_y=pose->y;
        pre_time = now_time;
        return odom_position;
    }

private:
     turtlesim::Pose *pose;
     geometry_msgs::PoseWithCovarianceStamped odom_position;
    double pre_time = ros::Time::now().toSec();
    double now_time = ros::Time::now().toSec();
    double pre_x=pose->x ;
    double pre_y=pose->y ;
    double del_time = 0.0;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cal_odom");
    ros::NodeHandle nh;

    ros::Subscriber position_sub = nh.subscribe("/turtle1/pose", 1, get_speedCallback);
    ros::Publisher odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/turtle1/odom", 1);
    ros::Duration(3).sleep();
    ros::spinOnce();
    calculate_odom calculate(&now_pose);
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        odom_pub.publish(calculate.update());
        loop_rate.sleep();
    }
    return 0;
}
