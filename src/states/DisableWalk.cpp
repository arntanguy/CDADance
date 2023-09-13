#include "DisableWalk.h"

#include <Eigen/src/Core/Matrix.h>
#include <mc_control/fsm/Controller.h>

#include "../WalkingInterface.h"

void DisableWalk::start(mc_control::fsm::Controller &ctl)
{
  auto &walk = *ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");

  walk.remove_stabilizer_task();

  output("OK");
  run(ctl);
}

bool DisableWalk::run(mc_control::fsm::Controller &ctl)
{
  return true;
}

void DisableWalk::teardown(mc_control::fsm::Controller &ctl)
{
}

EXPORT_SINGLE_STATE("DisableWalk", DisableWalk)
