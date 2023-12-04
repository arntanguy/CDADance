#include <mc_control/fsm/State.h>

struct Walk : mc_control::fsm::State {
  void start(mc_control::fsm::Controller &ctl) override;
  bool run(mc_control::fsm::Controller &ctl) override;
  void teardown(mc_control::fsm::Controller &ctl) override;

protected:
  bool walking_ = false;
  bool autoWalk_ = true;
  double stopDistance_ = 1;
  bool useStopDistance_ = true;
  Eigen::Vector3d startPos_;
};
