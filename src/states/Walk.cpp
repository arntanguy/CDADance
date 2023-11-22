#include "Walk.h"

#include <Eigen/src/Core/Matrix.h>
#include <mc_control/fsm/Controller.h>

#include "../WalkingInterface.h"

void Walk::start(mc_control::fsm::Controller &ctl)
{
  auto &walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");

  ctl.gui()->addElement(
      this, {},
      mc_rtc::gui::Label("Is walking?",
                         [&walk]()
                         { return walk.is_walking(); }),
      mc_rtc::gui::Button("START WALKING",
                          [this, &walk]()
                          {
                            walk.start_walking();
                            walking_ = true;
                          }),
      mc_rtc::gui::Button("STOP WALKING", [&walk]()
                          { walk.stop_walking(); }));

  config_("autoWalk", autoWalk_);
  /* config_("useStopDistance", useStopDistance_); */
  /* config_("stopDistance", stopDistance_); */
  /* Eigen::Vector3d refVel = Eigen::Vector3d::Zero(); */
  /* if (config_.has("refVel")) */
  /* { */
  /*   refVel = config_("refVel"); */
  /* } */
  if (config_.has("plan"))
  {
    mc_rtc::log::warning("PLAN IS SET");
    walk.load_plan(config_("plan"));
  }
  /* mc_rtc::log::info("refVel is {}", refVel); */
  // walk.set_planner_ref_vel(refVel);
  Eigen::Vector3d startPos_ = ctl.robot().posW().translation();
  if (autoWalk_)
  {
    walk.start_walking();
    mc_rtc::log::warning("ASKING TO START WALKING");
  }
  output("OK");
  run(ctl);
}

bool Walk::run(mc_control::fsm::Controller &ctl)
{
  auto &walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");
  if (!walking_)
  {
    if (!walk.is_walking())
      return false;
    else
      walking_ = true;
  }

  /* if (useStopDistance_ && (ctl.robot().posW().translation() -
   * startPos_).norm() >= stopDistance_) */
  /* { */
  /*   walk.set_planner_ref_vel(Eigen::Vector3d{0, 0, 0}); */
  /*   walk.stop_walking(); */
  /*   return true; */
  /* } */
  /* else */
  /* { */
  /* } */
  return walking_ && walk.is_stopped();
}

void Walk::teardown(mc_control::fsm::Controller &ctl)
{
  auto &walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");
  walk.set_planner_ref_vel(Eigen::Vector3d{0, 0, 0});
  walk.stop_walking();
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Walk", Walk)
