#include <human_traj_estimation/human_traj_estimation.h>
#include <human_traj_estimation/utils.h>
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>


TrajEstimator::TrajEstimator(ros::NodeHandle nh)
:nh_(nh)
{
  w_b_     .setZero();
  dW_      .setZero();
  velocity_.setZero();
  
  if ( !nh_.getParam ( "sampling_time", dt_) )
  {
    dt_=0.008;
    ROS_WARN_STREAM (nh_.getNamespace() << " /sampling_time set. default : "<<dt_);
  }
  
  if ( !nh_.getParam ( "K_tras", K_tras_) )
  {
    K_tras_=0.0001;
    ROS_WARN_STREAM (nh_.getNamespace() << " /K_tras set. default : " << K_tras_);
  }
  
  if ( !nh_.getParam ( "K_rot", K_rot_) )
  {
    K_rot_=0.000001;
    ROS_WARN_STREAM (nh_.getNamespace() << " /K_tras set. default : " << K_rot_);
  }

  if ( !nh_.getParam ( "base_link", base_link_) )
  {
    base_link_="base_link";
    ROS_WARN_STREAM (nh_.getNamespace() << " /base_link set. default : " << base_link_);
  }
  if ( !nh_.getParam ( "tool_link", tool_link_) )
  {
    tool_link_="tool_link";
    ROS_WARN_STREAM (nh_.getNamespace() << " /tool_link set. default : " << tool_link_);
  }

  ROS_INFO_STREAM (nh_.getNamespace() << " /K_tras set. default : " << K_tras_);
  ROS_INFO_STREAM (nh_.getNamespace() << " /K_rot set. default : " << K_rot_);
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    max_fl_ = 0.007;
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl set. default: " << max_fl_);
  }
  
  if ( !nh.getParam ( "alpha_switch", alpha_switch_) )
  {
    alpha_switch_ = 0.5;
    ROS_WARN_STREAM (nh.getNamespace() << " /alpha_switch_ not set. default " << alpha_switch_);
    
  alpha_ = 0.95;
  init_pos_ok = false;
  first_cb_ = false;

}

Eigen::Vector6d TrajEstimator::getVel() {return velocity_;}
Eigen::Vector6d TrajEstimator::getDwrench() {return dW_;}

void TrajEstimator::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg )
{  
  w_b_( 0 ) = msg->wrench.force.x;
  w_b_( 1 ) = msg->wrench.force.y;
  w_b_( 2 ) = msg->wrench.force.z;
  w_b_( 3 ) = msg->wrench.torque.x;
  w_b_( 4 ) = msg->wrench.torque.y;
  w_b_( 5 ) = msg->wrench.torque.z;
}

void TrajEstimator::alphaCallback(const std_msgs::Float32ConstPtr& msg )
{
  alpha_ = msg->data;
}

void TrajEstimator::dWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg )
{
  dW_( 0 ) = msg->wrench.force.x;
  dW_( 1 ) = msg->wrench.force.y;
  dW_( 2 ) = msg->wrench.force.z;
  dW_( 3 ) = msg->wrench.torque.x;
  dW_( 4 ) = msg->wrench.torque.y;
  dW_( 5 ) = msg->wrench.torque.z;
}


void TrajEstimator::velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg )
{
  velocity_( 0 ) = msg->twist.linear.x;
  velocity_( 1 ) = msg->twist.linear.y;
  velocity_( 2 ) = msg->twist.linear.z;
  velocity_( 3 ) = msg->twist.angular.x;
  velocity_( 4 ) = msg->twist.angular.y;
  velocity_( 5 ) = msg->twist.angular.z; 
}

void TrajEstimator::currPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg )
{
  cur_pos_ = *msg;
  
  if (!init_pos_ok)
  {
    ROS_INFO_STREAM("[human_traj_estimation] initializing pose for human desired trajectory update");
    init_pose_ = cur_pos_;
    last_pose_ = cur_pos_;
    init_pos_ok = true;
  }
  
  tf2::fromMsg (cur_pos_.pose, T_robot_base_targetpose_);
  
}


bool TrajEstimator::updatePoseEstimate(geometry_msgs::PoseStamped& ret)
{
  if (init_pos_ok)
  {
    
//     ret.pose.orientation = init_pose_.pose.orientation;
    if (alpha_>alpha_switch_)
      ret.pose = last_pose_.pose;
    else
      ret.pose = cur_pos_.pose;
    
    
    
    if(isnan(w_b_(0)))
      ROS_FATAL_STREAM("w_b_(0) : "<<w_b_(0));
    if(isnan(w_b_(1)))
      ROS_FATAL_STREAM("w_b_(1) : "<<w_b_(1));
    if(isnan(w_b_(2)))
      ROS_FATAL_STREAM("w_b_(2) : "<<w_b_(2));
    if(isnan(w_b_(3)))
      ROS_FATAL_STREAM("w_b_(3) : "<<w_b_(3));
    if(isnan(w_b_(4)))
      ROS_FATAL_STREAM("w_b_(4) : "<<w_b_(4));
    if(isnan(w_b_(5)))
      ROS_FATAL_STREAM("w_b_(5) : "<<w_b_(5));
    
    
    if (! isnan (w_b_(0)/std::fabs(w_b_(0))) )
    {
      ret.pose.position.x += K_tras_ * w_b_(0) ;
      ret.pose.position.y += K_tras_ * w_b_(1) ;
      ret.pose.position.z += K_tras_ * w_b_(2) ;
      
      double delta_z = K_rot_ * w_b_(5);
      
      Eigen::Quaterniond rotation_quaternion(T_robot_base_targetpose_.rotation());
      Eigen::Quaterniond additional_rotation_quaternion(Eigen::AngleAxisd(delta_z, Eigen::Vector3d::UnitZ()));
      rotation_quaternion = additional_rotation_quaternion * rotation_quaternion;
      tf2::convert(rotation_quaternion, ret.pose.orientation);
//       ROS_INFO_STREAM("pose:\n"<<ret);
      
    }
    last_pose_ = ret;
  }
  else
  {
    ROS_DEBUG_STREAM_THROTTLE(10.0,"[human_traj_estimation] initial pose for human desired trajectory update not yet initialized - not an issue if there is NO collaborative transport");
    return false;
  }
  
  return true;
}




// bool TrajEstimator::updateKestSrv(pbo_service::updateKest::Request  &req,
//                                   pbo_service::updateKest::Response &res)
// {
//   K_tras_ = req.K_assist;
//   last_pose_ = cur_pos_;
//   ROS_INFO_STREAM("K_tras updated ! new K_tras_: " << K_tras_);
//   res.res = true;
//   return true;
// }




bool TrajEstimator::resetPose(std_srvs::Trigger::Request  &req,
                              std_srvs::Trigger::Response &res)
{
  init_pos_ok = false;
//   tf::TransformListener listener_;
//   tf::StampedTransform transform_;
//   ROS_INFO_STREAM("reading transform from " << base_link_ << " to " << tool_link_);
//   listener_.waitForTransform(base_link_, tool_link_, ros::Time::now(), ros::Duration(1.0));
//   listener_.lookupTransform (base_link_, tool_link_, ros::Time(0)    , transform_);
//   
//   Eigen::Affine3d tmp;
//   tf::poseTFToEigen(transform_,tmp
//   tf::poseEigenToMsg (tmp, cur_pos_.pose);
  
  // TODO::might not be needed while, check
  while(!init_pos_ok)
  {
    ROS_INFO_STREAM_THROTTLE(1.0,"waiting for current pose to be read");
    ros::Duration(0.01);
  }
  init_pos_ok = true;
  
  init_pose_ = cur_pos_;
  last_pose_ = cur_pos_;
  
  ROS_INFO_STREAM("resetting estimation pose. new pose: \n" << init_pose_);
  
  res.success=true;
  return true;
}













