#include "XsensRetargetting.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/TransformTask.h>

void XsensRetargetting::start(mc_control::fsm::Controller &ctl)
{
  auto &ds = ctl.datastore();
  config_("stiffness", stiffness_);
  config_("weight", weight_);
  config_("fixed_stiffness", fixedStiffness_);
  config_("fixed_weight", fixedWeight_);
  config_("robot", robot_);
  config_("offset", offset_);
  config_("fixBaseLink", fixBaseLink_);
  config_("unactiveJoints", unactiveJoints_);
  config_("activeBodies", activeBodies_);
  if (config_.has("log"))
  {
    if (!ds.has("Replay::SetStartTime"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetStartTime does not exist in the Replay plugin");
    }
    if (!ds.has("Replay::SetEndTime"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetEndTime does not exist in the Replay plugin");
    }
    if (!ds.has("Replay::SetLog"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetLog does not exist in the Replay plugin");
    }
    const std::string &logName = config_("log");
    ds.call<void>("Replay::pause", config_("pause", false));
    ds.call<void>("Replay::SetStartTime", static_cast<double>(config_("start_time", 0.0)));
    if (config_.has("end_time"))
    {
      ds.call<void>("Replay::SetEndTime", static_cast<double>(config_("end_time")));
    }
    ds.call<void>("Replay::SetLog", logName);
  }

  if (robot_.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] \"robot\" parameter required");
  }
  else if (!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named \"{}\"");
  }
  auto &robot = ctl.robot(robot_);
  if (!ctl.config()("Xsens").has(robot.name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)", robot.name(), name(), robot.name());
  }
  if (!ctl.datastore().has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] This state requires the XsensPlugin", name());
  }

  ctl.gui()->addElement({"Xsens", robot_, "Offset base_link"},
                        mc_rtc::gui::ArrayInput("Offset Translation", offset_.translation()),
                        mc_rtc::gui::RPYInput("Offset RPY", offset_.rotation()));

  auto robotConfig = static_cast<std::map<std::string, mc_rtc::Configuration>>(ctl.config()("Xsens")(robot.name()));
  bool addActive = activeBodies_.empty();
  for (const auto &bodyConfig : robotConfig)
  {
    const auto &bodyName = bodyConfig.first;
    if (addActive) activeBodies_.push_back(bodyName);
    const auto &bodyConf = bodyConfig.second;
    bodyConfigurations_[bodyName] = XsensBodyConfiguration{};
    auto &bodyC = bodyConfigurations_[bodyName];
    bodyC.bodyName = bodyName;
    bodyC.segmentName = static_cast<std::string>(bodyConf("segment"));
    bodyConf("offset", bodyC.offset);
    if (bodyConf.has("weight"))
    {
      bodyC.weight = bodyConf("weight");
    }
    else
    {
      bodyC.weight = weight_;
    }
    if (bodyConf.has("stiffness"))
    {
      bodyC.stiffness = bodyConf("stiffness");
    }
    else
    {
      bodyC.stiffness = stiffness_;
    }

    if (isActiveBody(bodyName))
    {
      ctl.gui()->addElement({"Xsens", robot_, "Bodies", bodyName},
                            mc_rtc::gui::ArrayInput(
                                "Offset translation [m]",
                                [&bodyC]()
                                {
                                  return bodyC.offset.translation();
                                },
                                [&bodyC](const Eigen::Vector3d &offset)
                                {
                                  bodyC.offset.translation() = offset;
                                }),
                            mc_rtc::gui::ArrayInput(
                                "Offset rotation [rad]",
                                [&bodyC]()
                                {
                                  return mc_rbdyn::rpyFromMat(bodyC.offset.rotation());
                                },
                                [&bodyC](const Eigen::Vector3d &offset)
                                {
                                  bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset);
                                }));
    }
  }

  ctl.gui()->addElement({},
                        mc_rtc::gui::Button("Finished", [this]()
                                            { finished_ = true; }));

  stiffnessInterpolator_.values(
      {{0.0, config_("initialStiffness", 0.1)},
       {config_("initialStiffnessInterpolationDuration", 2.0), 1.}});

  // Initialize tasks
  for (auto &bodyName : activeBodies_)
  {
    const auto &body = bodyConfigurations_[bodyName];
    const auto &segmentName = body.bodyName;

    if (robot.hasBody(bodyName))
    {
      auto task = std::make_unique<mc_tasks::TransformTask>(robot.frame(bodyName), body.stiffness, body.weight);
      task->name(fmt::format("{}", bodyName));
      task->reset();
      task->selectUnactiveJoints(ctl.solver(), unactiveJoints_);
      ctl.solver().addTask(task.get());

      if (debugmode_)
      {
        mc_rtc::log::info("= = = = ctl.solver().addTask = = = =\n bodyName = {}, segmentName = {}\n", bodyName, segmentName);
      }

      tasks_[bodyName] = std::move(task);
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
    }
  }

  for (const auto &fixedBody : config_("fixedBodies", std::vector<std::string>{}))
  {
    mc_rtc::log::info("[{}] Fixed body: {}", fixedBody);
    auto task = std::make_unique<mc_tasks::TransformTask>(ctl.robot().frame(fixedBody), fixedStiffness_, fixedWeight_);
    task->reset();
    task->name(fmt::format("fixed_{}", fixedBody));
    task->selectUnactiveJoints(ctl.solver(), unactiveJoints_);
    ctl.solver().addTask(task.get());
    fixedTasks_[fixedBody] = std::move(task);
  }

  initPosW_ = robot.posW();
  output("OK");
  run(ctl);
}

bool XsensRetargetting::run(mc_control::fsm::Controller &ctl)
{
  auto & ds = ctl.datastore();
  auto &robot = ctl.robot(robot_);
  Eigen::VectorXd dimW = Eigen::VectorXd::Ones(6);

  std::string baseLinkSegment = "Pelvis";
  const auto baseLinkSegmentPose = bodyConfigurations_["body"].offset * ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>(baseLinkSegment));

  for (const auto &bodyName : activeBodies_)
  {
    const auto &body = bodyConfigurations_[bodyName];
    const auto &segmentName = body.segmentName;

    double percentStiffness = stiffnessInterpolator_.compute(t_);

    if (robot.hasBody(bodyName))
    {
      try
      {
        tasks_[bodyName]->dimWeight(dimW);
        tasks_[bodyName]->stiffness(percentStiffness * body.stiffness);

        const auto segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName);

        if (fixBaseLink_)
        {                                                                     // Apply all xsens MVN poses w.r.t a fixed initial robot base link
          auto X_blSP_segmentPose = segmentPose * baseLinkSegmentPose.inv();  // blSP: BaseLink_segmentationPose
          auto X_0_target = body.offset * X_blSP_segmentPose * offset_ * initPosW_;
          tasks_[bodyName]->target(X_0_target);
        }
        else
        {                                                                 // Directly apply world segment pose as obtained from w.r.t a fixed initial robot base linkom Xsens MVN
          tasks_[bodyName]->target(body.offset * segmentPose * offset_);  // change the target position
        }
      }
      catch (...)
      {
        mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
      }
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
    }
  }

  t_ += ctl.timeStep;
  return finished_ || ds.call<bool>("Replay::is_finished");
}

void XsensRetargetting::teardown(mc_control::fsm::Controller &ctl)
{
  for (const auto &task : tasks_)
  {
    ctl.solver().removeTask(task.second.get());
  }
  for (const auto &task : fixedTasks_)
  {
    ctl.solver().removeTask(task.second.get());
  }
}

EXPORT_SINGLE_STATE("XsensRetargetting", XsensRetargetting)
