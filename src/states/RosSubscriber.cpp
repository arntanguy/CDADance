#include "RosSubscriber.h"

#include <mc_rtc_ros/ros.h>

#include <iostream>

#include "../mc_lipm_stabilizer.h"

RosSubscriber::RosSubscriber() : nh_(mc_rtc::ROSBridge::get_node_handle()) {}

void RosSubscriber::chatterCallBack(const std_msgs::String::ConstPtr & msg)
{
  std::lock_guard<std::mutex> lock(receiveMutex_);
  rS_data.val = msg->data;
  ROS_INFO("I heard: [%s]", rS_data.val.c_str());
}

void RosSubscriber::rosSpinner()
{
  mc_rtc::log::info("ROS spinner thread created");

  ros::Rate rate(12);
  ros::Subscriber sub = nh_->subscribe("HPE_communicator_Winnie", 1000, &RosSubscriber::chatterCallBack, this);
  mc_rtc::log::info("[ROS listener]; topic = {}", sub.getTopic());

  while(active_ && ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  mc_rtc::log::info("ROS spinner destroyed");
}

void RosSubscriber::start(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().make<rosSubscriberData>("rosSubscriber_msg",
                                          rosSubscriberData{}); // init content of "rosSubscriber_msg" in the
                                                                // datastore to be empty
  spinThread_ = std::thread(std::bind(&RosSubscriber::rosSpinner, this));
}

bool RosSubscriber::run(mc_control::fsm::Controller & ctl)
{
  {
    std::lock_guard<std::mutex> lock(receiveMutex_);
    ctl.datastore().get<rosSubscriberData>("rosSubscriber_msg") = rS_data; // assign content
  }
  output("OK");
  return true;
}

void RosSubscriber::teardown(mc_control::fsm::Controller & ctl)
{
  if(ctl.datastore().has("rosSubscriber_msg"))
  {
    ctl.datastore().remove("rosSubscriber_msg");
  }
  active_ = false;
  spinThread_.join();
}

EXPORT_SINGLE_STATE("RosSubscriber", RosSubscriber)
