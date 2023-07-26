#include "../mc_lipm_stabilizer.h"
#include "RosSubscriber.h"
#include <iostream>


void chatterCallBack(const std_msgs::String::ConstPtr& msg)
{
    rS_data.val = msg->data;
    ROS_INFO("I heard: [%s]", rS_data.val.c_str());
}

void rosSpinner()
{
  ros::Rate rate(12);
  mc_rtc::log::info("ROS spinner thread created");

  // ros::spin();
  // rate.sleep();
  while(ros::ok())
  {
    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    ros::spinOnce();
    rate.sleep();
  }
  mc_rtc::log::info("ROS spinner destroyed");
}

void RosSubscriber::configure(const mc_rtc::Configuration & config)
{

}

void RosSubscriber::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);
}

bool RosSubscriber::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);

    if(active_)
    {
        ros::init(argc, argv, "listener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("HPE_communicator_Winnie", 1000, chatterCallBack);
        mc_rtc::log::info("[ROS listener]; topic = {}", sub.getTopic());
        spinThread_ = std::thread(&rosSpinner);

        if(ctl.datastore().has("rosSubscriber_msg"))
        {
            ctl.datastore().remove("rosSubscriber_msg");
        }
        auto & receivedata = ctl.datastore().make<rosSubscriberData>("rosSubscriber_msg", rS_data);
        mc_rtc::log::info("[ROS listener get string] = {}", receivedata.val);

        // spinThread_.join();
        spinThread_.detach();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else
    {
        //destroy the thread 
    }

    output("OK");
    return true;
}

void RosSubscriber::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}

EXPORT_SINGLE_STATE("RosSubscriber", RosSubscriber)
