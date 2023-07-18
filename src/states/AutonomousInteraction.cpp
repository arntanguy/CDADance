#include "AutonomousInteraction.h"
#include "../mc_lipm_stabilizer.h"
#include <time.h>
#include <iostream>
#include <cmath>

using namespace std;

void AutonomousInteraction::configure(const mc_rtc::Configuration & config)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::configure ------\n");}

  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);

  if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::configure ------\n");}

}

void AutonomousInteraction::start(mc_control::fsm::Controller & ctl_)
{
    if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::start ------\n");}

    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);

    if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::start ------\n");}
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

bool AutonomousInteraction::run(mc_control::fsm::Controller & ctl_)
{
  if(debugmode_){mc_rtc::log::info("------ DEBUG enter AutonomousInteraction::run ------\n");}

  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
  auto & robot = ctl.robot(robot_);

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("HPE_communicator_Winnie", 1000, chatterCallback);
  mc_rtc::log::info("------ [ROS listener]; topic = {}", sub.getTopic());
  ros::spin();

  // std::string SeqName[category_seq_] = {"SeqSF", "SeqAFL", "Seq3LookForApples", "Seq1Walk"};

  // mc_rtc::log::info("------ Return Index = {}; output = {} ------\n", segIdx, SeqName[segIdx]);
  // // output(SeqName[segIdx]);

  // if(debugmode_){mc_rtc::log::info("------ DEBUG leave AutonomousInteraction::run ------\n");}

  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}


EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)