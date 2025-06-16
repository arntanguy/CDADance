/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_lipm_stabilizer.h"

#include <mc_control/MCController.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include "WalkingInterface.h"
#include <lipm_walking/utils/SE2d.h>

// Patch the config so it's compatible with LIPMWalking
template<typename WalkingCtl>
static inline mc_rtc::Configuration patch_config(const mc_rbdyn::RobotModule & rm, mc_rtc::Configuration config)
{
  constexpr bool is_lipm = std::is_same_v<WalkingCtl, lipm_walking::Controller>;
  if constexpr(is_lipm)
  {
    // Attempt to load robot-specific config for LIPMWalking
    mc_rtc::Configuration conf;
    try
    {

      const auto lipm_robot_conf_path =
          std::string{mc_rtc::MC_CONTROLLER_INSTALL_PREFIX} + "LIPMWalking/" + rm.name + ".yaml";
      conf.load(lipm_robot_conf_path);
      mc_rtc::log::info(
          "Found robot-specific configuration for LIPMWalking for robot {} in {}: loading into the main configuration",
          rm.name, lipm_robot_conf_path);
      config.load(conf);
    }
    catch(mc_rtc::Configuration::Exception & e)
    {
      e.silence();
    }
  }

  return config;
}

static inline mc_rbdyn::RobotModulePtr patch_rm(mc_rbdyn::RobotModulePtr rm, const mc_rtc::Configuration & config)
{
  auto limits = config("Limits", mc_rtc::Configuration{})(rm->name, mc_rtc::Configuration{})
                    .operator std::map<std::string, mc_rtc::Configuration>();
  for(const auto & [joint, overwrite] : limits)
  {
    if(overwrite.has("lower"))
    {
      rm->_bounds[0].at(joint)[0] = overwrite("lower").operator double();
    }
    if(overwrite.has("upper"))
    {
      rm->_bounds[1].at(joint)[0] = overwrite("upper").operator double();
    }
  }
  return rm;
}

template<typename WalkingCtl>
struct WalkingInterfaceImpl : public WalkingInterface
{
  static constexpr bool is_lipm = std::is_same_v<WalkingCtl, lipm_walking::Controller>;
#ifdef WITH_ISMPC
  static constexpr bool is_ismpc = std::is_same_v<WalkingCtl, Walking_controller>;
#else
  static constexpr bool is_ismpc = false;
#endif

  static_assert(is_lipm || is_ismpc, "Write WalkingInterfaceImpl to support another walking base");

  WalkingInterfaceImpl(LIPMStabilizerController<WalkingCtl> & ctl) : ctl_(ctl) {}

  void load_plan(const std::string & name, const std::optional<sva::PTransformd> & relTarget)
  {
    if constexpr(is_lipm)
    {
      if(relTarget && (name == "custom_forward" || name == "custom_lateral" || name == "custom_backward"))
      {
        lipm_walking::utils::SE2d localTarget;
        localTarget.x = relTarget->translation().x();
        localTarget.y = relTarget->translation().y();
        localTarget.theta = mc_rbdyn::rpyFromMat(relTarget->rotation()).z();
        mc_rtc::log::info("load_plan {} with custom target (x: {}, y: {}, theta: {})", name, localTarget.x,
                          localTarget.y, mc_rtc::constants::toDeg(localTarget.theta));
        ctl_.planInterpolator.updateLocalTarget_(localTarget);
        ctl_.planInterpolator.run();
        ctl_.updatePlan(name);
      }
      else
      {
        ctl_.updatePlan(name);
      }
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      mc_rtc::log::error_and_throw("WalkingInterface: load_plan is not supported in ismpc");
    }
#endif
  }

  bool is_walking() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.walkingState == lipm_walking::WalkingState::SingleSupport
             || ctl_.walkingState == lipm_walking::WalkingState::DoubleSupport;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<bool>("ismpc_walking::robot_walking");
    }
#endif
    __builtin_unreachable();
  }

  bool is_double_support() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.walkingState == lipm_walking::WalkingState::DoubleSupport
             || ctl_.walkingState == lipm_walking::WalkingState::Standing;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<bool>("ismpc_walking::double_support");
    }
#endif
    __builtin_unreachable();
  }

  bool is_stopping() final
  {
    if constexpr(is_lipm)
    {
      return is_walking() && ctl_.pauseWalking;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<bool>("ismpc_walking::stop_phase");
    }
#endif
    __builtin_unreachable();
  }

  bool is_stopped() final
  {
    return !is_walking();
  }

  void start_walking()
  {
    if(is_stopped())
    {
      start_stop_walking();
    }
  }

  void stop_walking()
  {
    if(is_walking())
    {
      start_stop_walking();
    }
  }

  void start_stop_walking() final
  {
    mc_rtc::log::info("start_stop_walking");
    if constexpr(is_lipm)
    {
      mc_rtc::log::info("LIPM Start walking");
      if(ctl_.walkingState == lipm_walking::WalkingState::Standing)
      {
        if(!ctl_.startWalking)
        {
          mc_rtc::log::success("Start walking");
          ctl_.pauseWalking = false;
          ctl_.startWalking = true;
        }
      }
      else
      {
        if(!ctl_.pauseWalking)
        {
          mc_rtc::log::success("Stop walking");
          ctl_.pauseWalking = true;
        }
      }
      return;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<void>("ismpc_walking::start/stop");
    }
#endif
    __builtin_unreachable();
  }

  void set_planner_ref_vel(const Eigen::Vector3d & v) final
  {
    if constexpr(is_lipm)
    {
      if(ctl_.datastore().has("HybridPlanner::SetVelocity"))
      {
        auto & fn =
            ctl_.datastore().template get<std::function<void(const Eigen::Vector3d &)>>("HybridPlanner::SetVelocity");
        fn(v);
      }
      return;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<void, Eigen::Vector3d>("ismpc_walking::set_ref_vel", v);
    }
#endif
    __builtin_unreachable();
  }

  void set_torso_pitch(double p) final
  {
    if constexpr(is_lipm)
    {
      ctl_.stabilizer()->torsoPitch(p);
      return ctl_.plan.torsoPitch(p);
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<void, const double &>("ismpc_walking::set_torso_pitch", p);
    }
#endif
    __builtin_unreachable();
  }

  double get_com_height() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.plan.comHeight();
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      auto & cfg = ctl_.datastore().template call<ControllerConfiguration &>("ismpc_walking::get_config");
      return cfg.Stab_config.comHeight;
    }
#endif
    __builtin_unreachable();
  }

  void set_com_height(double h) final
  {
    if constexpr(is_lipm)
    {
      return ctl_.plan.comHeight(h);
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<void, const double &>("ismpc_walking::set_com_height", h);
    }
#endif
    __builtin_unreachable();
  }

  std::string get_support_foot() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.plan.supportContact().surfaceName;
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.datastore().template call<std::string>("ismpc_walking::support_foot_name");
    }
#endif
    __builtin_unreachable();
  }

  Eigen::Vector3d get_zmp_target() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.stabilizer()->targetZMP();
    }
#ifdef WITH_ISMPC
    if constexpr(is_ismpc)
    {
      return ctl_.MPCState().Pzk;
    }
#endif
    __builtin_unreachable();
  }

  void remove_stabilizer_task() final
  {
    if constexpr(is_lipm)
    {
      return ctl_.solver().removeTask(ctl_.stabilizer());
    }
  }

private:
  LIPMStabilizerController<WalkingCtl> & ctl_;
};

template<typename WalkingCtl>
LIPMStabilizerController<WalkingCtl>::LIPMStabilizerController(mc_rbdyn::RobotModulePtr rm,
                                                               double dt,
                                                               const mc_rtc::Configuration & config,
                                                               const mc_control::ControllerParameters & params)
: WalkingCtl(patch_rm(rm, config), dt, patch_config<WalkingCtl>(*rm, config), params)
{
  /* mc_rtc::log::info("FULL CONFIG IS {}",
   * mc_control::MCController::config().dump(true, true)); */

  walking_interface_ = std::make_shared<WalkingInterfaceImpl<WalkingCtl>>(*this);
  this->datastore().template make<WalkingInterfacePtr>("WalkingInterface", walking_interface_);

  this->gui()->addElement(
      this, {"CDADance"},
      mc_rtc::gui::Polygon("Stage Size",
                           [this]()
                           {
                             std::vector<Eigen::Vector3d> p;
                             p.emplace_back(stageMinSize_.x(), stageMinSize_.y(), 0.);
                             p.emplace_back(stageMinSize_.x(), stageMaxSize_.y(), 0.);
                             p.emplace_back(stageMaxSize_.x(), stageMaxSize_.y(), 0.);
                             p.emplace_back(stageMaxSize_.x(), stageMinSize_.y(), 0.);
                             p.emplace_back(stageMinSize_.x(), stageMinSize_.y(), 0.);
                             return p;
                           }),
      mc_rtc::gui::ArrayLabel(
          "Stage Width/Length", {"Width", "Length"}, [this]() -> std::array<double, 2>
          { return {stageMaxSize_.x() - stageMinSize_.x(), stageMaxSize_.y() - stageMinSize_.y()}; }),
      mc_rtc::gui::Button("Dump posture",
                          [this]()
                          {
                            mc_rtc::Configuration jc;
                            auto & robot = this->robot();
                            for(const auto & joint : robot.refJointOrder())
                            {
                              auto j = robot.mbc().q[robot.jointIndexByName(joint)];
                              if(j.size() == 1)
                              {
                                jc.add(joint, j[0]);
                              }
                            }
                            mc_rtc::log::info(jc.dump(true, true));
                          }));
}

template<typename WalkingCtl>
bool LIPMStabilizerController<WalkingCtl>::run()
{
  /* mc_rtc::Configuration jc; */
  /* auto &robot = this->robot(); */
  /* for (const auto &joint : robot.refJointOrder()) */
  /* { */
  /*   auto j = robot.mbc().q[robot.jointIndexByName(joint)]; */
  /*   if (j.size() == 1) */
  /*   { */
  /*     jc.add(joint, j[0]); */
  /*   } */
  /* } */
  /* mc_rtc::log::info(jc.dump(true, true)); */
  /* for(const auto & j : this->robot().mb().joints()) */
  /* { */
  /*   std::cout << j.name() << ", dof: " << j.dof() << std::endl; */
  /* } */
  auto & t = this->robot().posW().translation();
  stageMaxSize_.x() = std::max(t.x(), stageMaxSize_.x());
  stageMaxSize_.y() = std::max(t.y(), stageMaxSize_.y());
  stageMinSize_.x() = std::min(t.x(), stageMinSize_.x());
  stageMinSize_.y() = std::min(t.y(), stageMinSize_.y());
  return WalkingCtl::run();
}

/** Explicit instanciation of the controllers */
template struct LIPMStabilizerController<lipm_walking::Controller>;
#ifdef WITH_ISMPC
template struct LIPMStabilizerController<Walking_controller>;
#endif
