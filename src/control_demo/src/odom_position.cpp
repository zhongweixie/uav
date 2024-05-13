//头文件部分
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//初始化部分
geometry_msgs::PoseWithCovarianceStamped now_position;
geometry_msgs::PoseWithCovarianceStamped target_position;

void leg_odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  now_position=*msg;
  static geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = ros::Time::now();
  tfs.header.frame_id = "odom";
  tfs.child_frame_id = "base_link";
  tfs.transform.translation.x = now_position.pose.pose.position.x;
  tfs.transform.translation.y =now_position.pose.pose.position.y;
  tfs.transform.translation.z =now_position.pose.pose.position.z;
  tfs.transform.rotation=now_position.pose.pose.orientation;
   static tf2_ros::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tfs);
}
void target_position_odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    target_position=*msg;
}
class pid_control
{
    public:
      pid_control(double kp,double ki,double kd,double max,double min){
        KP=kp;
        KI=ki;
        KD=kd;
        MAX=max;
        MIN=min;  
      }
      double update(double error){
        double now_time = ros::Time::now().toSec();
        del_time=now_time-pre_time;
        integral=integral*0.99+pre_error*del_time; 
        double value = KP * error + KD * (error - pre_error) / del_time + KI * integral;
        if(value>MAX){
          value=MAX;
        }
        if(value<MIN){
          value=MIN;
        }
        
        pre_time=now_time;
        pre_error=error;
        return value;
      }
    private:
      double KP;
      double KI;
      double KD;
      double MAX;
      double MIN;
      double pre_error=0.0;
      double pre_time = ros::Time::now().toSec();
      double now_time = ros::Time::now().toSec();
      double integral = 0.0;
      double del_time = 0.0;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_position");
    ros::NodeHandle nh; 
    ros::Subscriber leg_odom_sub = nh.subscribe("turtle1/odom", 1, leg_odomCallback);
    ros::Subscriber target_position_sub = nh.subscribe("target_position_odom", 1, target_position_odomCallback);
    ros::Publisher  vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10); 
   
    now_position.pose.pose.position.x=0;
    now_position.pose.pose.position.y=0;
    now_position.pose.pose.position.z=0;
    now_position.pose.pose.orientation.w=1;
    now_position.pose.pose.orientation.x=0;
    now_position.pose.pose.orientation.y=0;
    now_position.pose.pose.orientation.z=0;
    target_position=now_position;
    ros::Rate loop_rate(50);
    pid_control x_pid_control(0.1,0.04,0.04,1.0,-1.0);
    pid_control y_pid_control(0.1,0.04,0.04,0.5,-0.5);
    pid_control yaw_pid_control(0.1,0.04,0.04,1.5,-1.5);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::Twist vel;
        bool chang_speed=true;
        if(chang_speed==true){
        double vx,vy,yaw;
               vx=x_pid_control.update(target_position.pose.pose.position.x-now_position.pose.pose.position.x);
              vy=y_pid_control.update(target_position.pose.pose.position.y-now_position.pose.pose.position.y);
              yaw=tf::getYaw(now_position.pose.pose.orientation);
              vel.linear.x=vx*cos(yaw)+vy*sin(yaw);
              vel.linear.y=vy*cos(yaw)-vx*sin(yaw);
        vel.angular.z=yaw_pid_control.update(tf::getYaw(target_position.pose.pose.orientation)-tf::getYaw(now_position.pose.pose.orientation));
        }else{
        vel.linear.x=x_pid_control.update(target_position.pose.pose.position.x-now_position.pose.pose.position.x);
        vel.linear.y=y_pid_control.update(target_position.pose.pose.position.y-now_position.pose.pose.position.y);
        vel.angular.z=yaw_pid_control.update(tf::getYaw(target_position.pose.pose.orientation)-tf::getYaw(now_position.pose.pose.orientation));
        }

        vel_pub.publish(vel); 
        loop_rate.sleep();
    }
    return 0;

}
