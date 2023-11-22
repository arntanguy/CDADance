/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "XsensReplay.h"

#include <mc_control/GlobalPluginMacros.h>

#include <mc_control/Ticker.h>
#include "mc_rtc/logging.h"

namespace mc_plugin
{

namespace
{

template<typename StrT>
std::string log_entry(const StrT & entry, const std::string & robot, bool is_main)
{
  if(is_main) { return entry; }
  return fmt::format("{}_{}", robot, entry);
}

/** Get a log entry for a robot from the given log at the given time */
template<typename GetT = std::vector<double>>
GetT get(const mc_rtc::log::FlatLog & log,
         const std::string & entry,
         const std::string & robot,
         bool is_main,
         size_t idx,
         const GetT & def = {})
{
  return log.get<GetT>(log_entry(entry, robot, is_main), idx, def);
}

/** Get the a robot's state from the log */
void log_to_robot(const mc_rtc::log::FlatLog & log, mc_rbdyn::Robot & robot, bool is_main, size_t idx)
{
  if(robot.mb().nrDof() == 0) { return; }
  auto qOut = get(log, "qOut", robot.name(), is_main, idx);
  for(size_t i = 0; i < qOut.size(); ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    if(mbcIdx == -1 || robot.mb().joint(mbcIdx).dof() == 0) { continue; }
    robot.mbc().q[static_cast<size_t>(mbcIdx)][0] = qOut[i];
  }
  if(robot.mb().joint(0).dof() == 6) { robot.posW(get<sva::PTransformd>(log, "ff", robot.name(), is_main, idx)); }
  else { robot.forwardKinematics(); }
}

template<typename CppT>
void update_datastore_fn(const mc_rtc::log::FlatLog & log,
                         const std::string & log_entry,
                         size_t idx,
                         mc_rtc::DataStore & ds,
                         const std::string & ds_entry)
{
  ds.assign(ds_entry, *log.getRaw<CppT>(log_entry, idx));
}

template<typename CppT>
void init_datastore(const mc_rtc::log::FlatLog & log,
                    const std::string & log_entry,
                    mc_rtc::DataStore & ds,
                    const std::string & ds_entry)
{
  ds.make<CppT>(ds_entry, *log.getRaw<CppT>(log_entry, 0));
}

XsensReplay::update_datastore_fn_t make_update_datastore_fn(const mc_rtc::log::FlatLog & log,
                                                       const std::string & log_entry,
                                                       mc_rtc::DataStore & ds,
                                                       const std::string & ds_entry)
{
  auto type = log.type(log_entry);
  switch(type)
  {
#define HANDLE_CASE(T)                                                     \
  case mc_rtc::log::LogType::T:                                            \
  {                                                                        \
    using CppT = mc_rtc::log::log_type_to_type_t<mc_rtc::log::LogType::T>; \
    init_datastore<CppT>(log, log_entry, ds, ds_entry);                    \
    return update_datastore_fn<CppT>;                                      \
  }
    HANDLE_CASE(Bool)
    HANDLE_CASE(Int8_t)
    HANDLE_CASE(Int16_t)
    HANDLE_CASE(Int32_t)
    HANDLE_CASE(Int64_t)
    HANDLE_CASE(Uint8_t)
    HANDLE_CASE(Uint16_t)
    HANDLE_CASE(Uint32_t)
    HANDLE_CASE(Uint64_t)
    HANDLE_CASE(Float)
    HANDLE_CASE(Double)
    HANDLE_CASE(String)
    HANDLE_CASE(Vector2d)
    HANDLE_CASE(Vector3d)
    HANDLE_CASE(Vector6d)
    HANDLE_CASE(VectorXd)
    HANDLE_CASE(Quaterniond)
    HANDLE_CASE(PTransformd)
    HANDLE_CASE(ForceVecd)
    HANDLE_CASE(MotionVecd)
    HANDLE_CASE(VectorDouble)
#undef HANDLE_CASE
    default:
      mc_rtc::log::error_and_throw("Cannot convert {} to C++ type automatically", LogTypeName(type));
  }
}

} // namespace

void XsensReplay::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  if(config.empty())
  {
    if(gc.controller().config().has("XsensReplay"))
    {
      auto replay_cfg = gc.controller().config()("XsensReplay");
      if(!replay_cfg.empty()) { return init(gc, replay_cfg); }
    }
    if(gc.configuration().config.has("XsensReplay"))
    {
      auto replay_cfg = gc.configuration().config("XsensReplay");
      if(!replay_cfg.empty()) { return init(gc, replay_cfg); }
    }
  }
  auto & ds = gc.controller().datastore();
  ds.make_call("Replay::SkipIter", [this](size_t skipIter) { skipIter_ = std::max(static_cast<size_t>(1), skipIter); });
  ds.make_call("Replay::SetStartTime",
               [this, &gc](double startTime) { start_iter_ = timeToIter(startTime, gc.timestep()); });
  ds.make_call("Replay::SetEndTime", [this, &gc](double endTime) { end_iter_ = timeToIter(endTime, gc.timestep()); });
  ds.make_call("Replay::GetStartTime", [this, &gc]() { return static_cast<double>(start_iter_) * gc.timestep(); });
  ds.make_call("Replay::GetCurrentTime", [this, &gc]() { return static_cast<double>(iters_) * gc.timestep(); });
  ds.make_call("Replay::GetEndTime", [this, &gc]() { return (static_cast<double>(end_iter_) - 1.0) * gc.timestep(); });
  ds.make_call("Replay::SetLog",
               [this, &ds, &gc](const std::string & logPath)
               {
                 init_log(logPath, ds);
                 reset(gc);
               });
  ds.make_call("Replay::SetPause", [this](bool pause) { pause_ = pause; });
  ds.make_call("Replay::IsFinished", [this]() { return finished_; });
  if(ds.has("Replay::Log")) { log_ = ds.get<decltype(log_)>("XsensReplay::Log"); }
  else
  {
    if(!config.has("log"))
    {
      mc_rtc::log::warning(
          "[XsensReplay] No log specified in the plugin configuration and no log available in the datastore at XsensReplay::Log");
    }
    else { init_log(config("log"), ds); }
  }
  std::string config_str;
  auto do_config = [&](const char * key, bool & check, std::string_view msg)
  {
    config(key, check);
    if(check)
    {
      if(config_str.size()) { config_str += ", "; }
      config_str += msg;
    }
  };
  do_config("with-inputs", with_inputs_, "replay sensor inputs");
  do_config("with-gui-inputs", with_gui_inputs_, "replay GUI inputs");
  do_config("with-outputs", with_outputs_, "replay controller output");
  do_config("pause", pause_, "start paused");
  start_iter_ = timeToIter(config("start_time", 0), gc.timestep());
  end_iter_ = timeToIter(config("end_time", 0), gc.timestep());
  auto preload_logs = config("preload_logs", std::vector<std::string>{});
  if(preload_logs.size())
  {
    for(const auto & logPath : preload_logs)
    {
      mc_rtc::log::info("[XsensReplay] Pre-loading {}", logPath);
      preload_logs_[logPath] = std::make_shared<mc_rtc::log::FlatLog>(logPath);
    }
  }
  if(pause_ && with_inputs_ && !with_outputs_)
  {
    mc_rtc::log::warning("[XsensReplay] Cannot start paused if only inputs are replayed");
    pause_ = false;
  }
  std::string with_datastore_config = config("with-datastore-config", std::string(""));
  if(!with_datastore_config.empty())
  {
    mc_rtc::log::info("[XsensReplay] Loading log to datastore configuration from {}", with_datastore_config);
    log_to_datastore_ = mc_rtc::Configuration(with_datastore_config).operator std::map<std::string, std::string>();
  }
  if(config_str.size()) { mc_rtc::log::info("[XsensReplay] Will {}", config_str); }
  else if(log_to_datastore_.empty()) { mc_rtc::log::warning("[XsensReplay] Configured to do nothing?"); }
  ctl_name_ = gc.controller().name_;
  reset(gc);
}

void XsensReplay::init_log(const std::string & logPath, mc_rtc::DataStore & ds)
{
  finished_ = false;
  if(preload_logs_.count(logPath) > 0)
  {
    mc_rtc::log::info("[XsensReplay] Loading log {} from preload cache", logPath);
    log_ = preload_logs_[logPath];
  }
  else
  {
    mc_rtc::log::info("[XsensReplay] Loading log {} from disk", logPath);
    log_ = std::make_shared<mc_rtc::log::FlatLog>(logPath);
  }
  if(!ds.has("XsensReplay::Log")) { ds.make<decltype(log_)>("XsensReplay::Log", log_); }
  if(log_->size() == 0) { mc_rtc::log::error_and_throw("[XsensReplay] Cannot replay an empty log"); }
  if(start_iter_ > log_->size() - 1)
  {
    mc_rtc::log::error_and_throw("[XsensReplay] Start time cannot be outside of the log");
  }
  else if(end_iter_ != 0 && end_iter_ > log_->size() - 1)
  {
    mc_rtc::log::warning("[XsensReplay] End time cannot be greater than end time");
    end_iter_ = log_->size() - 1;
  }
  if(end_iter_ == 0) { end_iter_ = log_->size() - 1; }
}

void XsensReplay::reset(mc_control::MCGlobalController & gc)
{
  iters_ = start_iter_;
  if(!log_) { return; }
  auto & ds = gc.controller().datastore();
  if(gc.controller().name_ != ctl_name_)
  {
    mc_rtc::log::warning(
        "[XsensReplay] Reset with a different controller than the initial one, jumping to the end of the log");
    iters_ = log_->size() - 1;
  }
  if(with_outputs_)
  {
    robots_ = mc_rbdyn::Robots::make();
    // Note: we copy the output robots here not the control robots
    gc.robots().copy(*robots_);
    for(const auto & r : *robots_)
    {
      gc.controller().gui()->removeElement({"Robots"}, r.name());
      gc.controller().gui()->addElement({"Robots"},
                                        mc_rtc::gui::Robot(r.name(), [&r]() -> const mc_rbdyn::Robot & { return r; }));
    }
  }
  // Initialize datastore
  for(const auto & ds_update : datastore_updates_)
  {
    if(ds.has(ds_update.ds_entry)) { ds.remove(ds_update.ds_entry); }
  }
  datastore_updates_.clear();
  for(auto it = log_to_datastore_.begin(); it != log_to_datastore_.end();)
  {
    const auto & [log_entry, ds_entry] = *it;
    if(!log_->has(log_entry))
    {
      mc_rtc::log::error("[XsensReplay] Requested to map {} to {} but {} is not in the log", log_entry, ds_entry, log_entry);
      it = log_to_datastore_.erase(it);
      continue;
    }
    datastore_updates_.push_back(
        {log_entry, ds_entry, make_update_datastore_fn(*log_, log_entry, gc.controller().datastore(), ds_entry)});
    ++it;
  }
  // Setup XsensReplay GUI
  gc.controller().gui()->removeCategory({"XsensReplay"});
  gc.controller().gui()->addElement(
      {"XsensReplay"},
      mc_rtc::gui::Button("Pause/Play",
                          [this]()
                          {
                            if(with_inputs_ && !with_outputs_)
                            {
                              mc_rtc::log::warning("[XsensReplay] XsensReplay cannot be paused when only inputs are replayed");
                              return;
                            }
                            pause_ = !pause_;
                          }));
  gc.controller().gui()->addElement(
      {"XsensReplay"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::NumberSlider(
          "XsensReplay time", [this, &gc]() { return static_cast<double>(iters_) * gc.timestep(); },
          [this, &gc](double t)
          {
            if(with_inputs_ && !with_outputs_)
            {
              mc_rtc::log::warning("[XsensReplay] XsensReplay time cannot be set when only inputs are replayed");
              return;
            }
            size_t iter = static_cast<size_t>(std::floor(t / gc.timestep()));
            iters_ = std::max<size_t>(std::min<size_t>(iter, log_->size() - 1), 0);
          },
          0.0, static_cast<double>(log_->size()) * gc.timestep()),
      mc_rtc::gui::Label("/", [this, &gc]() { return static_cast<double>(log_->size()) * gc.timestep(); }));

  // Use calibration from the replay
  if(with_inputs_ && log_->meta())
  {
    const auto & calibs = log_->meta()->calibs;
    for(const auto & [r, fs_calibs] : calibs)
    {
      if(!gc.robots().hasRobot(r)) { continue; }
      auto & robot = gc.robots().robot(r);
      for(const auto & [fs_name, calib] : fs_calibs)
      {
        auto & fs = const_cast<mc_rbdyn::ForceSensor &>(robot.forceSensor(fs_name));
        fs.loadCalibrator(mc_rbdyn::detail::ForceSensorCalibData::fromConfiguration(calib));
      }
    }
  }
  // Run once to fill the initial sensors
  before(gc);
}

void XsensReplay::before(mc_control::MCGlobalController & gc)
{
  if(!log_) { return; }
  const auto & log = *log_;
  if(with_inputs_)
  {
    for(const auto & r : gc.controller().robots())
    {
      bool is_main = r.name() == gc.controller().robot().name();
      // Restore joint level readings
      if(r.refJointOrder().size())
      {
        gc.setEncoderValues(r.name(), get(log, "qIn", r.name(), is_main, iters_));
        gc.setEncoderVelocities(r.name(), get(log, "alphaIn", r.name(), is_main, iters_));
        gc.setJointTorques(r.name(), get(log, "tauIn", r.name(), is_main, iters_));
      }
      // Restore force sensor readings
      std::map<std::string, sva::ForceVecd> wrenches;
      for(const auto & fs : r.forceSensors())
      {
        wrenches[fs.name()] = get(log, fs.name(), r.name(), is_main, iters_, sva::ForceVecd::Zero());
      }
      gc.setWrenches(r.name(), wrenches);
      // Restore body sensor readings
      std::map<std::string, Eigen::Vector3d> poses;
      mc_control::MCGlobalController::QuaternionMap oris;
      std::map<std::string, Eigen::Vector3d> linearVels;
      std::map<std::string, Eigen::Vector3d> angularVels;
      std::map<std::string, Eigen::Vector3d> linearAccels;
      std::map<std::string, Eigen::Vector3d> angularAccels;
      static auto def_quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      static Eigen::Vector3d def_vec = Eigen::Vector3d::Zero();
      for(const auto & bs : r.bodySensors())
      {
        poses[bs.name()] = get(log, bs.name() + "_position", r.name(), is_main, iters_, def_vec);
        oris[bs.name()] = get(log, bs.name() + "_orientation", r.name(), is_main, iters_, def_quat);
        linearVels[bs.name()] = get(log, bs.name() + "_linearVelocity", r.name(), is_main, iters_, def_vec);
        angularVels[bs.name()] = get(log, bs.name() + "_angularVelocity", r.name(), is_main, iters_, def_vec);
        linearAccels[bs.name()] = get(log, bs.name() + "_linearAcceleration", r.name(), is_main, iters_, def_vec);
        angularAccels[bs.name()] = get(log, bs.name() + "_angularAcceleration", r.name(), is_main, iters_, def_vec);
      }
      gc.setSensorPositions(r.name(), poses);
      gc.setSensorOrientations(r.name(), oris);
      gc.setSensorLinearVelocities(r.name(), linearVels);
      gc.setSensorAngularVelocities(r.name(), angularVels);
      gc.setSensorLinearAccelerations(r.name(), linearAccels);
      gc.setSensorAngularAccelerations(r.name(), angularAccels);
      // Restore joint sensor readings
      std::map<std::string, double> motorTemps;
      std::map<std::string, double> driverTemps;
      std::map<std::string, double> motorCurrents;
      for(const auto & js : r.jointSensors())
      {
        motorTemps[js.joint()] =
            get(log, "JointSensor_" + js.joint() + "_motorTemperature", r.name(), is_main, iters_, 0.0);
        driverTemps[js.joint()] =
            get(log, "JointSensor_" + js.joint() + "_driverTemperature", r.name(), is_main, iters_, 0.0);
        motorCurrents[js.joint()] =
            get(log, "JointSensor_" + js.joint() + "_motorCurrent", r.name(), is_main, iters_, 0.0);
      }
      gc.setJointMotorTemperatures(r.name(), motorTemps);
      gc.setJointDriverTemperatures(r.name(), driverTemps);
      gc.setJointMotorCurrents(r.name(), motorCurrents);
    }
  }
  if(with_gui_inputs_) { gc.server().push_requests(log.guiEvents()[iters_]); }
  for(auto & update_ds : datastore_updates_)
  {
    update_ds.update(*log_, update_ds.log_entry, iters_, gc.controller().datastore(), update_ds.ds_entry);
  }
}

void XsensReplay::after(mc_control::MCGlobalController & gc)
{
  if(!log_) { return; }
  if(with_outputs_)
  {
    for(auto & r : *robots_)
    {
      log_to_robot(*log_, r, r.name() == gc.controller().robot().name(), iters_);
      gc.robot(r.name()).mbc() = r.mbc();
    }
  }
  if(pause_ || finished_) {}
  else if(iters_ + 1 < end_iter_)
  {
    if(skipIter_ != 0 && iters_ != 0 && totalIter_ % skipIter_ == 0) { iters_++; }
    totalIter_++;
  }
  else
  {
    finished_ = true;
    end_iter_ = 0;
    start_iter_ = 0;
    totalIter_ = 0;
    skipIter_ = 1;
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("XsensReplay", mc_plugin::XsensReplay)
