#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>
#include <spdlog/common.h>

struct AutonomousInteraction : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  template<typename... Args>
  void debug_log(Args &&... args)
  {
    if(debugmode_)
    {
      mc_rtc::log::info(std::forward<Args>(args)...);
    }
  }

private:
  float rand_val;
  int remainder_val;
  int segIdx;
  int category_seq_ = 4;
  int capture_charatersN = 1;
  bool debugmode_ = true;
  std::vector<std::string> SeqName{"Seq0", "Seq1", "Seq2", "Seq3", "Seq4", "Seq5", "Seq6", "Seq7"};
  std::string subscriber_msg;
  bool finished_ = false;
};
