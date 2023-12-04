#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/ros.h>
#include <mc_rtc_ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <thread>

struct rosSubscriberData {
  std::string val{""};
};

struct RosSubscriber : mc_control::fsm::State {
  RosSubscriber();

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

protected:
  void chatterCallBack(const std_msgs::String::ConstPtr &msg);
  void rosSpinner();

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::atomic<bool> active_{
      true}; // start rosSunscriber on assigning true; while stop: false.
  std::thread spinThread_;
  rosSubscriberData rS_data;
  std::mutex receiveMutex_;
};
