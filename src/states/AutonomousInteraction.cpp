#include "AutonomousInteraction.h"

#include <time.h>

#include <cmath>
#include <iostream>

#include "../mc_lipm_stabilizer.h"
#include "RosSubscriber.h"

void AutonomousInteraction::configure(const mc_rtc::Configuration &config)
{
  debug_log("------ DEBUG enter AutonomousInteraction::configure ------\n");
  debug_log("------ DEBUG leave AutonomousInteraction::configure ------\n");
}

void AutonomousInteraction::start(mc_control::fsm::Controller &ctl_)
{
  debug_log("------ DEBUG enter AutonomousInteraction::start ------\n");

  run(ctl_);
  debug_log("------ DEBUG leave AutonomousInteraction::start ------\n");
}

bool AutonomousInteraction::run(mc_control::fsm::Controller &ctl)
{
  debug_log("------ DEBUG enter AutonomousInteraction::run ------\n");

  if (!ctl.datastore().has("rosSubscriber_msg"))
  {
    mc_rtc::log::error("No datastore value for rosSubscriber_msg");
  }

  auto &receivedMsgObj = ctl.datastore().get<rosSubscriberData>("rosSubscriber_msg");  // assign receivedMsgObj to false if cannot found "rosSubscriber_msg" in the datastore
  subscriber_msg = receivedMsgObj.val;
  debug_log("------ ReceivedMsgObj value = {} ------", subscriber_msg);

  bool find_msg = false;
  while (!find_msg)
  {
    if (subscriber_msg != "")
    {
      // extract le prediction label (0-7) from output str.
      segIdx = stoi(subscriber_msg.substr(subscriber_msg.find("world") + 6, capture_charatersN));
      find_msg = true;
    }
    else
    {
      mc_rtc::log::info("------ subscriber_msg = NULL ------\n");
      output("Null");
      return false;
    }
  }

  if (find_msg)
  {
    mc_rtc::log::info("------ Return Index = {}; output = {} ------\n", segIdx, SeqName[segIdx]);
    output(SeqName[segIdx]);
  }

  debug_log("------ DEBUG leave AutonomousInteraction::run ------\n");

  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller &ctl_)
{
}

EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)
