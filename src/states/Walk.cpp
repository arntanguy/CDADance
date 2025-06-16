#include "Walk.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/PTransform.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include "../WalkingInterface.h"
#include <lipm_walking/utils/SE2d.h>

void Walk::start(mc_control::fsm::Controller & ctl)
{
  auto & walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");

  ctl.gui()->addElement(this, {}, mc_rtc::gui::Label("Is walking?", [&walk]() { return walk.is_walking(); }),
                        mc_rtc::gui::Button("START WALKING",
                                            [this, &walk]()
                                            {
                                              walk.start_walking();
                                              walking_ = true;
                                            }),
                        mc_rtc::gui::Button("STOP WALKING", [&walk]() { walk.stop_walking(); }));

  config_("autoWalk", autoWalk_);
  if(auto planName = config_.find("plan"))
  {
    mc_rtc::log::warning("PLAN IS SET to {}", *planName);
    std::optional<sva::PTransformd> localTarget;
    if(auto localTargetConf = config_(*planName, mc_rtc::Configuration{}).find("localTarget"))
    {
      localTarget = sva::PTransformd::Identity();
      localTarget->translation().x() = (*localTargetConf)("x", 0.);
      localTarget->translation().y() = (*localTargetConf)("y", 0.);
      localTarget->rotation() = sva::RotZ(mc_rtc::constants::toRad((*localTargetConf)("theta", 0.)));
    }
    walk.load_plan(*planName, localTarget);
  }
  if(autoWalk_)
  {
    walk.start_walking();
    mc_rtc::log::warning("ASKING TO START WALKING");
  }

  startPos_ =
      sva::interpolate(ctl.robot().frame("LeftFoot").position(), ctl.robot().frame("RightFoot").position(), 0.5);
  output("OK");
  run(ctl);
}

bool Walk::run(mc_control::fsm::Controller & ctl)
{
  auto & walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");
  if(!walking_)
  {
    if(!walk.is_walking())
      return false;
    else
      walking_ = true;
  }

  return walking_ && walk.is_stopped();
}

void Walk::teardown(mc_control::fsm::Controller & ctl)
{
  auto & walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");
  walk.set_planner_ref_vel(Eigen::Vector3d{0, 0, 0});
  walk.stop_walking();
  ctl.gui()->removeElements(this);
  auto endPos =
      sva::interpolate(ctl.robot().frame("LeftFoot").position(), ctl.robot().frame("RightFoot").position(), 0.5);

  auto relWalkPose = endPos * startPos_.inv();
  mc_rtc::log::info("[{}] Walked:\n{}", name(),
                    lipm_walking::utils::SE2d{relWalkPose}.toConfiguration(true).dump(true, true));
}

EXPORT_SINGLE_STATE("Walk", Walk)
