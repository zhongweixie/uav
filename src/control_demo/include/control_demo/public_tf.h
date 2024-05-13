#ifndef _PUBLIC_TF_H  
#define _PUBLIC_TF_H  
#include <geometry_msgs/TransformStamped.h>
 #include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
 #include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
class PUBLIC_TF  
{  
    public:  
       PUBLIC_TF();  
       geometry_msgs::TransformStamped maptomap1;
       geometry_msgs::TransformStamped maptomap2;
       geometry_msgs::TransformStamped base_linktocamera_link;
      void pub_static(geometry_msgs::TransformStamped maptomap);
      void pub_dynamic(geometry_msgs::TransformStamped maptomap);
      tf2_ros::StaticTransformBroadcaster staticbroadcaster;
      tf2_ros::TransformBroadcaster dynamicbroadcaster;
};  
  
#endif 