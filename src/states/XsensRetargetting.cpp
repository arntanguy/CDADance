#include "XsensRetargetting.h"

#include <mc_control/fsm/Controller.h>

void XsensRetargetting::start(mc_control::fsm::Controller & ctl)
{
  auto & ds = ctl.datastore();
  config_("stiffness", stiffness_);
  config_("robot", robot_);
  config_("offset", offset_);
  config_("fixBaseLink", fixBaseLink_);
  config_("unactiveJoints", unactiveJoints_);
  if(config_.has("log"))
  {
    // FIXME start replay only here
  }

  if(debugmode_){mc_rtc::log::info("-----------------inside XsensRetargetting::start--------------");}

  if(robot_.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] \"robot\" parameter required");
  }
  else if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named \"{}\"");
  }
  auto & robot = ctl.robot(robot_);
  if(!ctl.config()("Xsens").has(robot.name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)", robot.name(), name(), robot.name());
  }
  if(!ctl.datastore().has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] This state requires the XsensPlugin", name());
  }
  
  auto robotConfig = static_cast<std::map<std::string, mc_rtc::Configuration>>(ctl.config()("Xsens")(robot.name()));
  for(const auto & bodyConfig : robotConfig)
  {
    const auto & bodyName = bodyConfig.first;
    const auto & bodyConf = bodyConfig.second;
    bodyConfigurations_[bodyName] = XsensBodyConfiguration{};
    auto & bodyC = bodyConfigurations_[bodyName];
    bodyC.bodyName = bodyName;
    bodyC.segmentName = static_cast<std::string>(bodyConf("segment"));
    bodyConf("offset", bodyC.offset);
    if(bodyConf.has("weight"))
    {
      bodyC.weight = bodyConf("weight");
    }
    else
    {
      bodyC.weight = weight_;
    }
    if(bodyConf.has("stiffness"))
    {
      bodyC.stiffness = bodyConf("stiffness");
    }
    else
    {
      bodyC.stiffness = stiffness_;
    }

    ctl.gui()->addElement({"Xsens", robot_, "Offset base_link"},
      mc_rtc::gui::ArrayInput("Offset Translation", offset_.translation()),
      mc_rtc::gui::RPYInput("Offset RPY", offset_.rotation())
    );

    ctl.gui()->addElement({"Xsens", robot_, "Bodies", bodyName},
      mc_rtc::gui::ArrayInput("Offset translation [m]",
      [&bodyC]()
      {
        return bodyC.offset.translation();
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.translation() = offset;
      }),
      mc_rtc::gui::ArrayInput("Offset rotation [rad]",
      [&bodyC]()
      {
        return mc_rbdyn::rpyFromMat(bodyC.offset.rotation());
      },
      [&bodyC](const Eigen::Vector3d & offset)
      {
        bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset);
      })
    );
  }

  ctl.gui()->addElement({},
    mc_rtc::gui::Button("Finished", [this]() { finished_ = true; }));

  // Initialize tasks
  for(auto & body: bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.bodyName;
      
    if(robot.hasBody(bodyName))
    {
      auto task = std::unique_ptr<mc_tasks::EndEffectorTask>(new mc_tasks::EndEffectorTask(bodyName, ctl.robots(), robot.robotIndex(), body.second.stiffness, body.second.weight));
      task->selectUnactiveJoints(ctl.solver(), unactiveJoints_);
      task->reset();
      ctl.solver().addTask(task.get());

      if(debugmode_){mc_rtc::log::info("= = = = ctl.solver().addTask = = = =\n bodyName = {}, segmentName = {}\n", bodyName, segmentName);}

      tasks_[bodyName] = std::move(task); 
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", bodyName);
    }    
  }
  initPosW_ = robot.posW();
  run(ctl);
}

bool XsensRetargetting::run(mc_control::fsm::Controller & ctl)
{
  auto & robot = ctl.robot(robot_);
  Eigen::VectorXd dimW = Eigen::VectorXd::Ones(6);

  std::string baseLinkSegment = "Pelvis";
  const auto baseLinkSegmentPose = bodyConfigurations_["body"].offset * ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>(baseLinkSegment));

  // if(debugmode_){mc_rtc::log::info("Pelvis pose called from datastore: {}; Pelvis offset in bodyConfiguration: {}\n", ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>(baseLinkSegment)), bodyConfigurations_["body"].offset);}

  for (const auto &body : bodyConfigurations_)
  {
    const auto & bodyName = body.first;
    const auto & segmentName = body.second.segmentName;

    if(robot.hasBody(bodyName))
    {
      try
      {
        tasks_[bodyName]->dimWeight(dimW);

        const auto segmentPose = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", segmentName); 

        if(fixBaseLink_)
        { // Apply all xsens MVN poses w.r.t a fixed initial robot base link
          auto X_blSP_segmentPose = segmentPose * baseLinkSegmentPose.inv(); //blSP: BaseLink_segmentationPose
          auto X_0_target = body.second.offset * X_blSP_segmentPose * offset_ * initPosW_;
          // mc_filter::utils::clampInPlace(X_0_target.translation().x(), initPosW_.translation().x() - 0.05, initPosW_.translation().x() + 0.3);
          tasks_[bodyName]->set_ef_pose(X_0_target);

        }
        else
        { // Directly apply world segment pose as obtained from w.r.t a fixed initial robot base linkom Xsens MVN
          tasks_[bodyName]->set_ef_pose(body.second.offset * segmentPose * offset_); //change the target position

        }
      }
      catch(...)
      {
        mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
      }
    }
    else
    {
      mc_rtc::log::error("[{}] No body named {}", name(), bodyName);
    }
  }
  output("OK");
  return finished_;
}

void XsensRetargetting::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & task : tasks_)
  {
    ctl.solver().removeTask(task.second.get());
  }
}

EXPORT_SINGLE_STATE("XsensRetargetting", XsensRetargetting)