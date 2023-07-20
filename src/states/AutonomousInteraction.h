#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_rtc/ros.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <thread>


struct AutonomousInteraction : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    float rand_val;
    int remainder_val;
    int segIdx;
    int category_seq_ = 4;
    int capture_charatersN = 1;
    double stiffness_ = 10;
    double weight_ = 1000;
    bool debugmode_ = false;
    std::string robot_{};
    sva::PTransformd offset_ = sva::PTransformd::Identity();
    std::thread spinThread_;

    int argc;
    char **argv;

};
   