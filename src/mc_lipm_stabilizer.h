/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#ifdef WITH_ISMPC
#include <ismpc_walking/Walking_controller.h>
#endif
#include <lipm_walking/Controller.h>
#include <mc_control/MCController.h>
#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Pause.h>
#include <mc_control/fsm/states/StabilizerStandingState.h>
#include <mc_control/mc_controller.h>

#include "WalkingInterface.h"

template <typename WalkingCtl>
struct MC_CONTROL_DLLAPI LIPMStabilizerController : public WalkingCtl
{
  LIPMStabilizerController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration& config, const mc_control::ControllerParameters& params = mc_control::ControllerParameters{});
  bool run() override;

 protected:
  WalkingInterfacePtr walking_interface_;
};

extern template struct LIPMStabilizerController<lipm_walking::Controller>;
#ifdef WITH_ISMPC
extern template struct LIPMStabilizerController<Walking_controller>;
#endif
