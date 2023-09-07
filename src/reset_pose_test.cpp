#include "ros/ros.h"
#include <configuration_msgs/StartConfiguration.h>
#include <std_srvs/Trigger.h>
#include <thread>


static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";


ros::ServiceClient reset_pose_estimation_;
void reset_pose_call() 
{
  ROS_INFO_STREAM(CYAN<<"resetting human estimation pose ");
  std_srvs::Trigger reset_pose;
  reset_pose_estimation_.call(reset_pose);
  ROS_INFO_STREAM(CYAN<<"pose reset !");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  
  ros::Rate r = 10;
    
  ros::ServiceClient configuration_srv = nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  reset_pose_estimation_ = nh.serviceClient<std_srvs::Trigger>("/reset_pose_estimation");
    
  ROS_INFO_STREAM("[  ]" << " waitin for server /configuration_manager/start_configuration");
  configuration_srv.waitForExistence();
  ROS_INFO_STREAM("[  ]" << " /configuration_manager/start_configuration connected ! ");
  ROS_INFO_STREAM("[  ]" << " waitin for server /reset_pose_estimation");
  reset_pose_estimation_.waitForExistence();
  ROS_INFO_STREAM("[  ]" << " /reset_pose_estimation connected ! ");

  
  while(ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(5.0,"[  ]" << " - looping");

    std_srvs::Trigger reset_pose;
    configuration_msgs::StartConfiguration start;
    
    int conf = 0;
    ROS_INFO_STREAM("enter configuration desired - 0: planner - 1: drapebot_integration");
    std::cin >> conf;
    
    if (conf==1)
    {
      ROS_INFO_STREAM(BOLDYELLOW<<"setting drapebot_test");
      std::thread t(reset_pose_call);
      ros::Duration(0.1).sleep();
      start.request.start_configuration = "drapebot_integration";
      start.request.strictness = 1;
      configuration_srv.call(start);
      t.join();
    }
    else if (conf == 0)
    {
      ROS_INFO_STREAM(BOLDWHITE<<"setting planner config");
      start.request.start_configuration = "planner";
      start.request.strictness = 1;
      configuration_srv.call(start);
    }
    else
    {
      ROS_INFO_STREAM(BOLDWHITE<<"setting planner config");
    }
      
    
    r.sleep();    
    
  }

  return 0;
}

