#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include <Eigen/Dense>

 
 
void callBack(const geometry_msgs::PoseStamped callBackData){ 
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  Eigen::AngleAxisd rotation_vector1 ( M_PI/2, Eigen::Vector3d ( 0,0,1 ) );  //3x1 , rotate x axix at 90 degree
  //Eigen::AngleAxisd rotation_vector2 ( M_PI/2, Eigen::Vector3d ( 0,0,1 ) ); 
  Eigen::Vector4d dd; //4x1
  dd = Eigen::Quaterniond( rotation_vector1  ).coeffs();
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(dd[0], dd[1], dd[2], dd[3]) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "aruco_marker_582_frame", "new_co"));
} 
 
int main(int argc, char** argv){
  ros::init(argc, argv, "transform_broadcaster");
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/aruco_tracker/pose", 1, callBack);

  ros::spin();
  return 0;
};

