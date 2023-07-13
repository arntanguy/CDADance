#include "AutonomousInteraction.h"
#include "../mc_lipm_stabilizer.h"
#include <time.h>
#include <iostream>
#include <cmath>

using namespace std;

void AutonomousInteraction::configure(const mc_rtc::Configuration & config)
{
  config("stiffness", stiffness_);
  config("robot", robot_);
  config("offset", offset_);
}

void AutonomousInteraction::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
    run(ctl);
}

bool AutonomousInteraction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
  auto & robot = ctl.robot(robot_);

  std::string SeqName[category_seq_] = {"SeqSF", "SeqAFL", "Seq3LookForApples", "Seq1Walk"};

  srand( (unsigned)time(NULL) );
  rand_val = 10 * (float) rand()/RAND_MAX;
  remainder_val = int(floor(rand_val)) % category_seq_;

  mc_rtc::log::info("------ Random Value = {}; output {} ------\n", remainder_val, SeqName[remainder_val]);
  output(SeqName[remainder_val]);

  return true;
}

void AutonomousInteraction::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<LIPMStabilizerController &>(ctl_);
}


EXPORT_SINGLE_STATE("AutonomousInteraction", AutonomousInteraction)