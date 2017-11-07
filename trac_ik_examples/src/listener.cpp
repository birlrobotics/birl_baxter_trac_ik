#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
using namespace std;


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<geometry_msgs::Pose>("aruco_marker_change", 10);
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/base", "/new_co",  ros::Time(0), ros::Duration(10.0) );
      //ROS_INFO("get the transform");
      listener.lookupTransform("/base",  "/new_co",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::Pose msg;
    msg.position.x = transform.getOrigin().x();
    msg.position.y = transform.getOrigin().y();
    msg.position.z = transform.getOrigin().z();
    msg.orientation.x = transform.getRotation().x();
    msg.orientation.y = transform.getRotation().y();
    msg.orientation.z = transform.getRotation().z();
    msg.orientation.w = transform.getRotation().w();

    pub.publish(msg);

    rate.sleep();
  }
  return 0;
};
