/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_lipm_stabilizer.h"

extern "C" {
CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> &names) {
  CONTROLLER_CHECK_VERSION("CDADance")
  names.emplace_back("CDADance");
#ifdef WITH_ISMPC
  names.emplace_back("CDADance_ismpc");
#endif
}

CONTROLLER_MODULE_API void destroy(mc_control::MCController *ptr) {
  delete ptr;
}

CONTROLLER_MODULE_API unsigned int create_args_required() { return 4; }

CONTROLLER_MODULE_API mc_control::MCController *
create(const std::string &name, const mc_rbdyn::RobotModulePtr &robot,
       const double &dt, const mc_control::Configuration &conf) {
  if (name == "CDADance") {
    return new LIPMStabilizerController<lipm_walking::Controller>(robot, dt,
                                                                  conf);
  }
#ifdef WITH_ISMPC
  if (name == "CDADance_ismpc") {
    return new LIPMStabilizerController<Walking_controller>(
        robot, dt, conf,
        mc_control::ControllerParameters{}
            .load_robot_config_into({})
            .overwrite_config(true));
  }
#endif
  mc_rtc::log::error("This library cannot create a controller named {}", name);
  return nullptr;
}
}
