#include <human_traj_estimation/human_traj_estimation.h>
#include <human_traj_estimation/utils.h>
#include <ros/package.h>

TrajEstimator::TrajEstimator(ros::NodeHandle nh)
:nh_(nh)
{
  w_b_     .setZero();
  dW_      .setZero();
  velocity_.setZero();
  
  if ( !nh_.getParam ( "sampling_time", dt_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /sampling_time set. default 0.008");
    dt_=0.008;
  }
  
  std::string path = ros::package::getPath("human_traj_estimator");
  path += "/config/assistance.fis";
  ROS_INFO_STREAM("recovering path: " << path);
  
  engine_ = fl::FisImporter().fromFile(path);
  std::string status;
  if (not engine_->isReady(&status))
    ROS_ERROR_STREAM("engine is not ready");

  d_force_   = engine_->getInputVariable("dforce");
  vel_     = engine_->getInputVariable("velocity");
  assistance_= engine_->getOutputVariable("assistance");
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    max_fl_ = 0.007;
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl set. default: " << max_fl_);
  }
  
  alpha_ = 0.95;
  init_pos_ok = false;

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
    init_pose_ = cur_pos_;
    last_pose_ = cur_pos_;
    init_pos_ok = true;
  }
}


double TrajEstimator::evaluateFis( double dforce, double vel )
{
  if(dforce>20)
    dforce=19;
  if(vel >0.3)
    vel=0.29;
  
  d_force_->setValue(dforce);
  vel_->setValue(vel);
  engine_->process();
  
  double out = assistance_->getValue();
  
  ROS_INFO_STREAM_THROTTLE(0.2,GREEN<<"dforce : "<<d_force_->getValue()<<", vel: "<<vel_->getValue());
  
  if ( isnan(out) )
  {
    out = 0;
    ROS_WARN_STREAM_THROTTLE(2.0,"setting assistance to max: "<< out<< " with dforce: "<<d_force_->getValue()<<", and vel "<< vel_->getValue() );
  }
  
//   if ( ( out > alpha_max_ ) )
//   {
//     out = alpha_max_ ;
//     CNR_INFO_THROTTLE(this->logger(),2.0,"saturating alpha to max: "<<out);
//   }
//   else if ( out < alpha_min_  )
//   {
//     out = alpha_min_;
//     CNR_INFO_THROTTLE(this->logger(),2.0,"saturating alpha to min: "<<out);
//   }    
  
  return out;
}



bool TrajEstimator::updatePoseEstimate(geometry_msgs::PoseStamped& ret)
{
  if (init_pos_ok)
  {
    ret.pose.orientation = init_pose_.pose.orientation;
    if (alpha_>0.5)
      ret.pose.position = last_pose_.pose.position;
    else
      ret.pose.position = cur_pos_.pose.position;
    
    if (! isnan (w_b_(0)/std::fabs(w_b_(0))) )
    {
      ret.pose.position.x += 0.0001 * w_b_(0) ;
      ret.pose.position.y += 0.0001 * w_b_(1) ;
      ret.pose.position.z += 0.0001 * w_b_(2) ;
      
//     if (! isnan (w_b_(0)/std::fabs(w_b_(0))) )
//       ret.pose.position.x += ( 0.00001 * std::fabs(dW_(0)) * w_b_(0)/std::fabs(w_b_(0)) ) + ( 0.001 * velocity_(0) );
//     if (! isnan (w_b_(1)/std::fabs(w_b_(1))) )
//       ret.pose.position.y += ( 0.00001 * std::fabs(dW_(1)) * w_b_(1)/std::fabs(w_b_(1)) ) + ( 0.001 * velocity_(1) );
//     if (! isnan (w_b_(2)/std::fabs(w_b_(2))) )
//       ret.pose.position.z += ( 0.00001 * std::fabs(dW_(2)) * w_b_(2)/std::fabs(w_b_(2)) ) + ( 0.001 * velocity_(2) );
    }
    
    
//     double al_x = evaluateFis( std::fabs( dW_(0) ) ,std::fabs( velocity_(0) ) );
//     double al_y = evaluateFis( std::fabs( dW_(1) ) ,std::fabs( velocity_(1) ) );
//     double al_z = evaluateFis( std::fabs( dW_(2) ) ,std::fabs( velocity_(2) ) );
//     
//     ROS_INFO_STREAM_THROTTLE(0.2,"al_x : "<<al_x <<", al_y : "<<al_y <<", al_z : "<<al_z);
//     
//     if (! isnan (dW_(0)/std::fabs(dW_(0))) )
//       ret.pose.position.x += 0.1 * al_x * dW_(0)/std::fabs(dW_(0));
//     if (! isnan (dW_(1)/std::fabs(dW_(1))) )
//       ret.pose.position.y += 0.1 * al_y * dW_(1)/std::fabs(dW_(1));
//     if (! isnan (dW_(2)/std::fabs(dW_(2))) )
//       ret.pose.position.z += 0.1 * al_z * dW_(2)/std::fabs(dW_(2));
//     
//     ROS_INFO_STREAM_THROTTLE(0.2,"vx : "<<dW_(0)/std::fabs(dW_(0))<<", v_y : "<<dW_(1)/std::fabs(dW_(1))<<", v_z : "<<dW_(2)/std::fabs(dW_(2)));
    
    last_pose_ = ret;
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1.0,"pose not initialized !");
    return false;
  }
  
  return true;
}

















