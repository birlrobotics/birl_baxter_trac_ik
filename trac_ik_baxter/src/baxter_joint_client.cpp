#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trac_ik_baxter/GetConstrainedPositionIK.h>
#include <baxter_core_msgs/EndpointState.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>

//geometry_msgs::PoseStamped pose[];


int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_right_arm_joint_pub");
  ros::NodeHandle n;
  trac_ik_baxter::GetConstrainedPositionIK srv;
  ros::ServiceClient client = n.serviceClient<trac_ik_baxter::GetConstrainedPositionIK>("trac_ik_right");
  
 // ros::Subscriber sub = n.subscribe("endpoint_pose", 1, Callback);
  

  ros::Time curr_stamp(ros::Time::now());  
  std::string base;
  //srv.request.pose_stamp = new geometry_msgs::PoseStamped;
  //srv.request.pose_stamp.back
  geometry_msgs::PoseStamped temp,temp1;


 // temp.header.frame_id = base;
  //temp.header.stamp = curr_stamp;
  temp.pose.position.x = 0.316787;
  temp.pose.position.y=  0.117263;
  temp.pose.position.z = 0.0302719;
  temp.pose.orientation.x = 0.479594;
  temp.pose.orientation.y = 0.833593;
  temp.pose.orientation.z= -0.0911262;
  temp.pose.orientation.w = 0.258475;


  srv.request.pose_stamp.push_back(temp);
  srv.request.num_steps = 100;
  srv.request.end_tolerance = 0.1;
  //srv.request.seed_angles.push_back(temp1);
  
  
  
 /* srv.request.pose_stamp[0].header.frame_id = base;
  srv.request.pose_stamp[0].header.stamp = curr_stamp;
  
  srv.request.pose_stamp[0].pose.position.x = 0.855273;
  srv.request.pose_stamp[0].pose.position.y= 0.913992;
  srv.request.pose_stamp[0].pose.position.z = 2.482157;
  srv.request.pose_stamp[0].pose.orientation.x = 1.044758;
  srv.request.pose_stamp[0].pose.orientation.y = -0.183486;
  srv.request.pose_stamp[0].pose.orientation.z= -1.535637;
  srv.request.pose_stamp[0].pose.orientation.w = 0.447512;*/
  
  ROS_INFO("fill the request");
  
  if (client.call(srv))
  {
    ROS_INFO("succeed");
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  return 0;
}
  

