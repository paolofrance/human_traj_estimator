#pragma once

#include <ros/ros.h>
#include <fl/Headers.h>
#include <rosdyn_core/primitives.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <rotations_helper/euler_angles_helper.h>
// #include <pbo_service/updateKest.h>
#include <std_srvs/Trigger.h>

class TrajEstimator
{
public:
  TrajEstimator(ros::NodeHandle nh);
  
  Eigen::Vector6d getVel(); 
  Eigen::Vector6d getDwrench(); 
  
  void wrenchCallback  (const geometry_msgs::WrenchStampedConstPtr& msg );
  void alphaCallback   (const std_msgs::Float32ConstPtr& msg );
  void dWrenchCallback (const geometry_msgs::WrenchStampedConstPtr& msg );
  void velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg );
  void currPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg );
//   double evaluateFis   (double dforce, double vel );
  bool updatePoseEstimate(geometry_msgs::PoseStamped& ret); 
  bool resetPose(std_srvs::Trigger::Request  &req,
                 std_srvs::Trigger::Response &res);
  
//   bool updateKestSrv( pbo_service::updateKest::Request  &req,
//                       pbo_service::updateKest::Response &res);
  
  EulerAnglesHelper actual_rot_;
  double init_rot_;
  
  Eigen::Affine3d T_robot_base_targetpose_;
  bool init_pos_ok;

private:
  ros::NodeHandle nh_;
  
  rosdyn::ChainPtr chain_bt_;
  ros::Publisher al_pub_;
  
  Eigen::Vector6d w_b_;
  Eigen::Vector6d velocity_;
  Eigen::Vector6d dW_;
  
  bool first_cb_;
  
  double alpha_switch_;
  
  geometry_msgs::PoseStamped cur_pos_;
  geometry_msgs::PoseStamped init_pose_;
  geometry_msgs::PoseStamped last_pose_;
  
  double K_tras_;
  double K_rot_;
  
  std::string tool_link_;
  std::string base_link_;
  
  double max_fl_;
  double min_fl_;
  double alpha_;
  
  double dt_;
/*  
  fl::Engine*         engine_;
  fl::InputVariable*  d_force_ ;
  fl::InputVariable*  vel_ ;
  fl::OutputVariable* assistance_ ;
  */
  
};

