#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "kdl/frames.hpp"
#include <cmath>
using namespace std;
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
#include <Eigen/Dense>



std::string chain_start, chain_end, urdf_param;
double timeout = 3;
double eps = 1e-3;
ros::Publisher jointCommandPublisher;
double joint_state[6];


// Computes IK with limits considerations. 
// Inputs: KDL::Frame        pose
//         const std::string limbName 
// Output: publishes solution.
void findIKSolution(const KDL::Frame &end_effector_pose, const std::string &limbName)
{

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, limbName + chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;                           // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);
  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  nominal(0) = (ll(0)+ul(0))/2;
  nominal(1) = (ll(1)+ul(1))/2;
  nominal(2) = (ll(2)+ul(2))/2;
  nominal(3) = (ll(3)+ul(3))/2;
  nominal(4) = (ll(4)+ul(4))/2;
  nominal(5) = (ll(5)+ul(5))/2;
  nominal(6) = (ll(6)+ul(6))/2;
  ROS_INFO("get starting joint states");
  
	
  KDL::JntArray result;
  // result count
  int rc;

  rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
  
  ROS_INFO("Found %d solution", rc);

  if (rc > 0) {
      /* In the solution, the order of joint angles is s->e->w,
       * while baxter_core_msgs::JointCommand requires the order of e->s->w.
       * So we have to make some conversions.
      */
      ROS_INFO("get in rc");
      std::string jointNamesArray[] = {limbName + "_s0", limbName + "_s1", limbName + "_e0", limbName + "_e1",
          limbName + "_w0", limbName + "_w1", limbName + "_w2"};
      std::vector<std::string> jointNamesVector(chain.getNrOfJoints());
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j)
          jointNamesVector[j] = jointNamesArray[j];
      baxter_core_msgs::JointCommand jointCommand;
      jointCommand.mode = 1;
      jointCommand.names = jointNamesVector;
      jointCommand.command.resize(chain.getNrOfJoints());
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j)
          jointCommand.command[j] = result(j);

      // The conversion has done, publishing the joint command
      ROS_INFO("Joint_space: q0=%f, q1=%f, q2=%f, q3=%f, q4=%f, q5=%f, q6=%f",
     result.data[0], result.data[1], result.data[2], 
     result.data[3], result.data[4], result.data[5], 
     result.data[6]);
     jointCommandPublisher.publish(jointCommand);
  }
}

//----------------------------------------------------------------------------------------------

//----------------Get the joint space ---------------------------------------------

// this callback function is for test whether can we get the IK solver
void test_by_given_hard_code (const baxter_core_msgs::EndpointState &callBackData2){
    ROS_INFO("test ik");
    geometry_msgs::Pose temp;
    temp.position.x = 0.273476;
    temp.position.y = -0.338266;
    temp.position.z = 0.515312;
    temp.orientation.x = -0.034;
    temp.orientation.y = 0.999;
    temp.orientation.z = -0.029;
    temp.orientation.w = -0.003;
    


   /* temp.position.x = callBackData2.pose.position.x;
    temp.position.y = callBackData2.pose.position.y;
    temp.position.z = callBackData2.pose.position.z + 0.005;
    temp.orientation.x = callBackData2.pose.orientation.x;
    temp.orientation.y = callBackData2.pose.orientation.y;
    temp.orientation.z = callBackData2.pose.orientation.z;
    temp.orientation.w = callBackData2.pose.orientation.w; */
    KDL::Frame end_effector_pose;
    tf::poseMsgToKDL(temp, end_effector_pose);
    // Compute the IK Solution and publish the solution from within the method
    findIKSolution(end_effector_pose, "right");
  }
     
void changed_marker_pose(const geometry_msgs::Pose &callBackData){	
  ROS_INFO("get the listener information");
    geometry_msgs::Pose temp;
   
    temp.position.x = callBackData.position.x;
    temp.position.y = callBackData.position.y;
    temp.position.z = callBackData.position.z;
    temp.orientation.x = callBackData.orientation.x;
    temp.orientation.y = callBackData.orientation.y;
    temp.orientation.z = callBackData.orientation.z;
    temp.orientation.w = callBackData.orientation.w;

  KDL::Frame end_effector_pose;
  cout << "marker_pose"<<endl;
  cout << callBackData;
  tf::poseMsgToKDL(temp, end_effector_pose);
    // Compute the IK Solution and publish the solution from within the method
  findIKSolution(end_effector_pose, "right");
  
}


// This callback function is for get current position of robot
void current_joint_state (const sensor_msgs::JointState &callBackData1){
  ROS_INFO("get current joint states");
  joint_state[0] = callBackData1.position[9];
  joint_state[1] = callBackData1.position[10];
  joint_state[2]=  callBackData1.position[11];
  joint_state[3] = callBackData1.position[12];
  joint_state[4] = callBackData1.position[13];
  joint_state[5] = callBackData1.position[14];
  joint_state[6] = callBackData1.position[15];
}


// Compute the Inverse Kinematics 
void initial_marker_pose (const geometry_msgs::PoseStamped &callBackData) {
 
  ROS_INFO("Callbacking...");
  geometry_msgs::Pose temp;
 
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  Eigen::AngleAxisd rotation_vector1 ( M_PI/2, Eigen::Vector3d ( 1,0,0 ) );  //3x1 , rotate x axix at 90 degree
  Eigen::AngleAxisd rotation_vector2 ( M_PI/2, Eigen::Vector3d ( 0,0,1 ) );      // then rotate z axix at 180 degree
  Eigen::Matrix3d rotation_matrix1,rotation_matrix2, rotation_matrix3, temp_rotation1, temp_rotation2;
  
  Eigen::Vector4d dd; //4x1
  dd = Eigen::Quaterniond( rotation_vector1 * rotation_vector2 ).coeffs();
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(dd[0], dd[1], dd[2], dd[3]) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "aruco_marker_582_frame", "new_co"));

  
  rotation_matrix1 = rotation_vector1.toRotationMatrix();  // 3x3 matrix
  rotation_matrix2 = rotation_vector2.toRotationMatrix(); 
  rotation_matrix3 = rotation_matrix2 * rotation_matrix1  ;
  
  temp.position.x = callBackData.pose.position.x;
  temp.position.y = callBackData.pose.position.y;
  temp.position.z = callBackData.pose.position.z;
  temp.orientation.x = callBackData.pose.orientation.x;
  temp.orientation.y = callBackData.pose.orientation.y;
  temp.orientation.z = callBackData.pose.orientation.z;
  temp.orientation.w = callBackData.pose.orientation.w;
  
  cout<<"--------- origin pose-----------------";
  cout<<endl;
  cout << temp<<endl;
  
  Eigen::Quaterniond q1(temp.orientation.x, temp.orientation.y, temp.orientation.z, temp.orientation.w);
  

  Eigen::Matrix3d R1 = q1.toRotationMatrix();
  cout << "convert quat to 3x3 matrix"<< endl;  
  cout << R1 << endl;
  
  temp_rotation1 = rotation_matrix1 * R1    ;
  cout << endl;
  cout<<"first rotation with x axis at 90 degree ";
  cout << endl;
  cout<< temp_rotation1;
  cout << endl;
  
  Eigen::Vector4d tt1;
  tt1 = Eigen::Quaterniond(temp_rotation1).coeffs();
  tt1.normalize();
  cout<<"----convert 3x3 matrix to quat----------------------";
  cout << endl;
  cout << tt1 << endl;
  
  temp.orientation.x = tt1[0];
  temp.orientation.y = tt1[1];
  temp.orientation.z = tt1[2];
  temp.orientation.w = tt1[3];
  
  cout << "after first orientaion we get the temp " << endl;
  cout<< temp; 
  
  
//------------------------------------------------------------  
  
  /*Eigen::Quaterniond q2(temp.orientation.x, temp.orientation.y, temp.orientation.z, temp.orientation.w);
  Eigen::Matrix3d R2 = q2.toRotationMatrix();
  temp_rotation2 =  R2 * rotation_matrix2    ;
  
  
  Eigen::Vector4d tt2;
  tt2 = Eigen::Quaterniond(temp_rotation2).coeffs();

  
  temp.orientation.x = tt2[0];
  temp.orientation.y = tt2[1];
  temp.orientation.z = tt2[2];
  temp.orientation.w = tt2[3];  
  cout<<"---after second orientaion we get the temp ---------";
  cout <<endl;
  cout << temp;
  temp.orientation.x = -0.045983;
  temp.orientation.y = 0.989218;
  temp.orientation.z = 0.0298579;
  temp.orientation.w = 0.1167816;*/
  
  KDL::Frame end_effector_pose;
  tf::poseMsgToKDL(temp, end_effector_pose);
    // Compute the IK Solution and publish the solution from within the method
  findIKSolution(end_effector_pose, "right");
  }



int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "go_to_position");
  ros::NodeHandle nodeHandle;
  chain_start = "base";
  chain_end = "_gripper_base";
  urdf_param = "/robot_description";
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }


  
  //ros::Subscriber subscriber = nodeHandle.subscribe("/aruco_tracker/pose", 1, initial_marker_pose);   // this is from initial mark pose
  
  ros::Subscriber subscriber = nodeHandle.subscribe("/aruco_marker_change", 1, changed_marker_pose);  // because the marker is facing up, we need change the pose to adapte to robot hand pose
  
  
  ros::Subscriber subscriber1 = nodeHandle.subscribe("/robot/joint_states", 1, current_joint_state); // get current joint states for giving a initial state
  
 //ros::Subscriber subscriber2 = nodeHandle.subscribe("/robot/limb/right/endpoint_state", 1, test_by_given_hard_code); // just for testing
  

  jointCommandPublisher = nodeHandle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10); // send command to robot to let it run to the pose we give to it

  ros::spin();
  return 0;
}
