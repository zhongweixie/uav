#include <geometry_msgs/TransformStamped.h>
  #include <control_demo/public_tf.h>
  #include <ros/ros.h>
  #include <tf2_ros/transform_broadcaster.h>
 #include <tf2_ros/static_transform_broadcaster.h>
 #include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#define M_PI 3.14159265358979323846
PUBLIC_TF::PUBLIC_TF()  
{  
    geometry_msgs::Quaternion  q_zero;
    q_zero.w=1;
    q_zero.x=0;
    q_zero.y=0;
    q_zero.z=0;

    maptomap1.header.frame_id="map";
    maptomap1.child_frame_id="map1";
    maptomap1.transform.translation.x=2.0;
    maptomap1.transform.translation.y=0.0;
    maptomap1.transform.translation.z=0.0;
    maptomap1.transform.rotation=q_zero;

    maptomap2.header.frame_id="map";
    maptomap2.child_frame_id="map2";
    maptomap2.transform.translation.x=2.0;
    maptomap2.transform.translation.y=2.0;
    maptomap2.transform.translation.z=0.0;
    maptomap2.transform.rotation=q_zero;
    //相机前向安装时的变换
    base_linktocamera_link.header.frame_id="base_link";
    base_linktocamera_link.child_frame_id="camera_link";
    base_linktocamera_link.transform.translation.x=0.0;
    base_linktocamera_link.transform.translation.y=0.0;
    base_linktocamera_link.transform.translation.z=0.0;

  geometry_msgs::Quaternion quat;
  tf::Quaternion tf_quat;
  //前视摄像头
  tf_quat.setRPY(-90.0* M_PI / 180.0,0.0 * M_PI / 180.0,-90.0* M_PI / 180.0);
  //下视摄像头
  //tf_quat.setRPY(180* M_PI / 180.0,0.0 * M_PI / 180.0,-90.0* M_PI / 180.0);
  tf::quaternionTFToMsg(tf_quat, quat);
  base_linktocamera_link.transform.rotation=quat;
    

    staticbroadcaster.sendTransform(maptomap1);
    staticbroadcaster.sendTransform(maptomap2);
    staticbroadcaster.sendTransform(base_linktocamera_link);
}  

void PUBLIC_TF::pub_static(geometry_msgs::TransformStamped maptomap){

    staticbroadcaster.sendTransform(maptomap);
}
void PUBLIC_TF::pub_dynamic(geometry_msgs::TransformStamped maptomap){

    dynamicbroadcaster.sendTransform(maptomap);
}
