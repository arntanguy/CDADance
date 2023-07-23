#include <mc_control/fsm/State.h>
#include <mc_rtc/ros.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <thread>


// struct RosSubscriberConfiguration
// {
    
// }
struct rosSubscriberData
{
    std::string val{};
};

rosSubscriberData rS_data;

void chatterCallBack(const std_msgs::String::ConstPtr& msg);
void rosSpinner();


struct RosSubscriber:mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    bool active_ = true; //start rosSunscriber on assigning true; while stop: false.
    std::thread spinThread_;
    int argc;
    char **argv;

};