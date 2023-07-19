#include "AutonomousInteraction.h"
#include "../mc_lipm_stabilizer.h"
#include <time.h>
#include <iostream>
#include <cmath>

using namespace std;
std::string subscriber_msg = "";

void AutonomousInteraction::configure(const mc_rtc::Configuration & config)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::configure ------\n");}

  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);

  if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::configure ------\n");}

}

void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  subscriber_msg = msg->data;
  mc_rtc::log::info(" chatterCallback DEBUG INFO subscriber_msg:  {}", subscriber_msg);
  ROS_INFO("I heard: [%s]", subscriber_msg.c_str());
}

void AutonomousInteraction::start(mc_control::fsm::Controller & ctl_)
{
    if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::start ------\n");}

    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);

    if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::start ------\n");}
}

bool AutonomousInteraction::run(mc_control::fsm::Controller & ctl_)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::run ------\n");}

  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
  auto & robot = ctl.robot(robot_);
  std::string SeqName[4] = {"SeqSF", "SeqAFL", "Seq3LookForApples", "Seq1Walk"};

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("HPE_communicator_Winnie", 1000, chatterCallback);
  mc_rtc::log::info("------ [ROS listener]; topic = {}", sub.getTopic());
  ros::Rate rate(24);
  bool find_msg = false;

  while(ros::ok())
  {
    if(subscriber_msg != "")
    {
      segIdx = stoi(subscriber_msg.substr(subscriber_msg.find("world") + 6, capture_charatersN)); 

      find_msg = true;
    }
    else
    {
      mc_rtc::log::info("------ subscriber_msg = NULL ------\n");
    }

    if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::run ------\n");}

    ros::spinOnce();
    // ros::spin();
    rate.sleep();

    if(find_msg){
      mc_rtc::log::info("------ INSIDE TEMINAL msg log ------\n");
      break;
    }  
  }

  mc_rtc::log::info("------ Return Index = {}; output = {} ------\n", segIdx, SeqName[segIdx]);
  output(SeqName[segIdx]);
  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}


EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)