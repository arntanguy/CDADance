#include "InterpolatePosture.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/PostureTask.h>

#include <algorithm>
#include <iostream>
#include <random>

#include "../WalkingInterface.h"

void InterpolatePosture::start(mc_control::fsm::Controller &ctl)
{
  robotName_ = config_("robot", ctl.robot().name());
  if (!ctl.hasRobot(robotName_))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), robotName_);
  }
  auto &robot = ctl.robot(robotName_);
  if (!config_.has(robot.name()))
  {
    mc_rtc::log::error_and_throw("[{}] No configuration for robot {}", name(), robot.name());
  }
  auto robotConfig = config_(robot.name());

  // Parse the desired posture sequence.
  // We expect a vector of PostureConfig
  // Example in yaml:
  //  posture_sequence:
  //    - time: 0.0
  //      posture:
  //        R_SHOULDER_P: -0.25
  //        L_SHOULDER_P: -0.21
  //        R_ELBOW_P: -0.55
  //        L_ELBOW_P: -0.20
  //    - time: 2.0
  //      posture:
  //        L_SHOULDER_R: 1.38
  //        L_SHOULDER_Y: 1.22
  //      shake:
  //        NECK_Y:
  //          period: 0.1 # s
  //          amplitude: 1 # rad
  config_("autoplay", autoplay_);
  config_("repeat", repeat_);
  config_("useDefaultPostureTask", useDefaultPostureTask_);
  config_("restorePostureGains", restorePostureGains_);
  config_("goBackToInitialPosture", goBackToInitialPosture_);
  config_("usePostureTransitionCriteria", usePostureTransitionCriteria_);
  config_("postureTransitionSpeed", postureTransitionSpeed_);
  config_("enableShake", enableShake_);
  config_("enableLookAt", enableLookAt_);
  config_("improvise", improvise_);
  robotConfig("autoplay", autoplay_);
  robotConfig("improvise", improvise_);
  robotConfig("goBackToInitialPosture", goBackToInitialPosture_);
  robotConfig("useDefaultPostureTask", useDefaultPostureTask_);
  robotConfig("restorePostureGains", restorePostureGains_);
  robotConfig("usePostureTransitionCriteria", usePostureTransitionCriteria_);
  robotConfig("postureTransitionSpeed", postureTransitionSpeed_);
  robotConfig("enableShake", enableShake_);
  robotConfig("enableLookAt", enableLookAt_);
  if (ctl.datastore().has("Improvise"))
  {
    improvise_ = ctl.datastore().get<bool>("Improvise");
    ctl.datastore().remove("Improvise");
  }
  robotConfig("repeat", repeat_);

  std::vector<PostureConfig> postureSequence = robotConfig("posture_sequence");

  // If improvising, shuffle order
  if (improvise_)
  {
    std::random_device rd;
    std::mt19937 g{rd()};
    std::shuffle(postureSequence.begin(), postureSequence.end(), g);
  }

  // Get the list of actuated joints

  const auto &rjo = robot.refJointOrder();
  // Start interpolation from current posture
  PostureConfig initPosture;
  initPosture.t = 0.0;
  for (const auto &jName : rjo)
  {
    if (robot.hasJoint(jName))
    {
      const auto jIdx = robot.jointIndexByName(jName);
      if (robot.mb().joint(jIdx).dof() == 1)
      {
        initPosture.posture[jName] = robot.mbc().q[jIdx][0];
      }
    }
  }
  postureSequence_.push_back(initPosture);

  mc_rtc::log::critical("here");
  // Create a vector used to store the desired value for each actuated joint
  Eigen::VectorXd desiredPosture(rjo.size());
  double t = 0;
  // Initialize with current robot posture
  for (int i = 0; i < rjo.size(); ++i)
  {
    const auto jIdx = robot.jointIndexInMBC(i);
    if (robot.mb().joint(jIdx).dof() == 1)
    {
      desiredPosture(i) = robot.mbc().q[jIdx][0];
    }
  }

  mc_rtc::log::critical("there");

  for (auto postureConfig : postureSequence)
  {
    t += postureConfig.t;
    postureConfig.t = t;
    if (postureConfig.halfsitting)
    {
      for (const auto &jName : robot.refJointOrder())
      {
        const auto &hsStance = robot.module().stance();
        const auto jIdx = robot.jointIndexByName(jName);
        if (robot.hasJoint(jName) && robot.mb().joint(jIdx).dof() == 1)
        {
          postureConfig.posture[jName] = hsStance.at(jName)[0];
        }
      }
    }
    postureSequence_.push_back(postureConfig);
  }
  mc_rtc::log::critical("there2");

  // Last posture should be the init posture no matter what
  if (goBackToInitialPosture_)
  {
    t += 2.0;
    initPosture.t = t;
    postureSequence_.push_back(initPosture);
  }

  // Convert to absolute time
  double scaleTime = robotConfig("scaleTime", 1.);
  if (robotConfig.has("duration"))
  {
    scaleTime = static_cast<double>(robotConfig("duration")) / postureSequence_.back().t;
  }
  for (auto &postureConfig : postureSequence_)
  {
    postureConfig.t = postureConfig.t * scaleTime;
  }

  // Create the interpolator values
  PostureInterpolator::TimedValueVector interpolatorValues;
  interpolatorValues.emplace_back(0.0, desiredPosture);

  CoMInterpolator::TimedValueVector comInterpolatorValues;
  comInterpolatorValues.emplace_back(0.0, Eigen::Vector3d::Zero());

  // For each timed posture in the sequence
  for (const auto &postureConfig : postureSequence_)
  {
    const auto &postureMap = postureConfig.posture;
    // For each actuated joint
    for (int i = 0; i < rjo.size(); ++i)
    {
      const auto &actuatedJoint = rjo[i];
      const auto jIdx = robot.jointIndexByName(rjo[i]);
      if (!robot.hasJoint(actuatedJoint) || robot.mb().joint(jIdx).dof() != 1) continue;
      // Check if we have a desired posture in the configuration
      if (postureMap.count(actuatedJoint))
      {
        // If so, put the desired joint value for this actuated joint
        desiredPosture(i) = postureMap.at(actuatedJoint);
      }
      else
      {
        // Otherwise use the current joint value
        desiredPosture(i) = robot.mbc().q[robot.jointIndexInMBC(i)][0];
      }
    }
    // Add the current posture to the interpolator values
    interpolatorValues.emplace_back(postureConfig.t, desiredPosture);
    comInterpolatorValues.emplace_back(postureConfig.t, postureConfig.comOffset);
  }
  mc_rtc::log::critical("there3");
  // Put all desired postures in the interpolator
  interpolator_.values(interpolatorValues);
  comInterpolator_.values(comInterpolatorValues);

  // Load the configuration for the posture task.
  // See https://jrl-umi3218.github.io/mc_rtc/json.html#MetaTask/PostureTask for
  // supported values
  // Example in yaml:
  //   posture_task:
  //     stiffness: 100
  if (useDefaultPostureTask_)
  {
    postureTask_ = ctl.getPostureTask(robot.name());
  }
  else
  {
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(ctl.solver(), robot.robotIndex());
    ctl.solver().addTask(postureTask_);
  }
  initialPostureStiffness_ = postureTask_->stiffness();
  initialPostureWeight_ = postureTask_->weight();
  postureTask_->load(ctl.solver(), robotConfig("posture_task", mc_rtc::Configuration{}));
  postureTask_->reset();

  lookAt_ = std::make_shared<mc_tasks::LookAtTask>(ctl.robot().frame("NECK_P_LINK"), Eigen::Vector3d{1, 0, 0}, 10.0, 100.0);

  ctl.gui()->addElement(
      this, {name()},
      mc_rtc::gui::Checkbox(
          "Play", [this]()
          { return autoplay_; },
          [this]()
          { autoplay_ = !autoplay_; }),
      mc_rtc::gui::Checkbox(
          "Update posture", [this]()
          { return updatePosture_; },
          [this]()
          { updatePosture_ = !updatePosture_; }),
      mc_rtc::gui::NumberInput(
          "Time", [this]()
          { return t_; },
          [this](double t)
          { t_ = t; }));

  ctl.gui()->addElement(
      this, {name()}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::NumberSlider(
          "Time selector", [this]()
          { return t_; },
          [this](double t)
          { t_ = t; },
          0,
          interpolator_.values().back().first),
      mc_rtc::gui::Label("/", [this]()
                         { return postureSequence_.back().t; }));

  ctl.gui()->addElement(
      this, {name()},
      mc_rtc::gui::Checkbox(
          "Repeat Motion", [this]()
          { return repeat_; },
          [this]()
          { repeat_ = !repeat_; }),
      mc_rtc::gui::Checkbox(
          "Improvise next time",
          [this, &ctl]()
          {
            return ctl.datastore().has("Improvise") && ctl.datastore().get<bool>("Improvise");
          },
          [this, &ctl]()
          {
            if (!ctl.datastore().has("Improvise"))
            {
              ctl.datastore().make<bool>("Improvise", true);
            }
            else
            {
              ctl.datastore().remove("Improvise");
            }
          }),
      mc_rtc::gui::Checkbox(
          "Improvising?", [this]()
          { return improvise_; },
          [this]() {}),
      mc_rtc::gui::Checkbox(
          "Enable Shake", [this]()
          { return enableShake_; },
          [this]()
          { enableShake_ = !enableShake_; }),
      mc_rtc::gui::Checkbox(
          "Enable LookAt", [this]()
          { return enableLookAt_; },
          [this]()
          { enableLookAt_ = !enableLookAt_; }));
  run(ctl);
}

bool InterpolatePosture::run(mc_control::fsm::Controller &ctl)
{
  auto &robot = ctl.robot(robotName_);
  const auto &rjo = robot.refJointOrder();

  // Compute the interpolated posture at the current time
  auto desiredPosture = interpolator_.compute(t_);
  auto desiredCoMOffset = comInterpolator_.compute(t_);

  // Shake
  auto currPostureSeq =
      std::find_if(postureSequence_.begin(), postureSequence_.end(), [this](const auto &p)
                   { return p.t > t_; });
  // currPostureSeq--;
  if (currPostureSeq != postureSequence_.end())
  {
    if (enableShake_)
    {
      const auto &shakeMap = currPostureSeq->shake;
      // mc_rtc::log::info("Should shake (t={}, posture t= {})", t_, currPostureSeq->t);
      // mc_rtc::log::info("Joints: {}", mc_rtc::io::to_string(shakeMap, [](const auto & m) { return m.first; }));
      // For each actuated joint
      for (int i = 0; i < rjo.size(); ++i)
      {
        // Shake
        const auto &actuatedJoint = rjo[i];
        const auto jIdx = robot.jointIndexByName(actuatedJoint);
        if (shakeMap.count(actuatedJoint) && robot.mb().joint(jIdx).dof() == 1)
        {
          const auto &shakeConfig = shakeMap.at(actuatedJoint);
          // Shake value such that it starts with
          // - Shake = 0 for t_ = currPostureSeq->t (no motion initially)
          // - It shakes with period shakeConfig.period around the current joint
          // trajectory value
          // - It shakes with amplitude shakeConfig.amplitude
          double shakeVal =
              shakeConfig.amplitude * sin(2 * mc_rtc::constants::PI / shakeConfig.period * (t_ - currPostureSeq->t));
          desiredPosture(i) += shakeConfig.direction * shakeVal;
          // mc_rtc::log::info("Shaking joint {} : {}", actuatedJoint, shakeVal);
        }
      }
    }

    // LookAt task
    if (enableLookAt_)
    {
      const auto &lookAtConfig = currPostureSeq->lookAt;
      if (lookAtConfig)
      {
        auto lookRobot = lookAtConfig->robot ? *lookAtConfig->robot : ctl.robot().name();
        lookAt_->target(ctl.robot(lookRobot).frame(lookAtConfig->frame).position().translation());
        if (!lookAtActive_)
        {
          lookAt_->stiffness(lookAtConfig->stiffness);
          lookAt_->weight(lookAtConfig->weight);
          ctl.solver().addTask(lookAt_);
          lookAtActive_ = true;
        }
      }
      else
      {
        if (lookAtActive_)
        {
          ctl.solver().removeTask(lookAt_);
          lookAtActive_ = false;
        }
      }
    }
  }

  // Get the posture task
  auto &postureTask = *ctl.getPostureTask(robot.name());
  // Copy the current posture target
  auto posture = postureTask_->posture();

  // For each actuated joint
  for (int i = 0; i < rjo.size(); ++i)
  {
    const auto &actuatedJoint = rjo[i];
    const auto jIdx = robot.jointIndexByName(rjo[i]);
    if (robot.mb().joint(jIdx).dof() != 1) continue;
    // Set the posture target for this actuated joint to its interpolated value
    posture[jIdx][0] = desiredPosture[i];
  }

  // Change the posture target in the posture task
  if (updatePosture_)
  {
    postureTask_->posture(posture);
  }

  if (updateCoM_)
  {
    auto desiredCoMHeight = robot.module()._lipmStabilizerConfig.comHeight + desiredCoMOffset.z();
    if (ctl.datastore().has("StabilizerStandingState::setCoMTarget"))
    {
      auto comTarget = ctl.datastore().call<const Eigen::Vector3d &>("StabilizerStandingState::getCoMTarget");
      comTarget.z() = desiredCoMHeight;
      ctl.datastore().call<void>("StabilizerStandingState::setCoMTarget",
                                 static_cast<const Eigen::Vector3d &>(comTarget));
    }
    else if (ctl.datastore().has("WalkingInterface"))
    {
      auto walk = ctl.datastore().get<WalkingInterfacePtr>("WalkingInterface");
      walk->set_com_height(desiredCoMHeight);
    }
  }

  if (autoplay_)
  {
    t_ += ctl.timeStep;
  }

  bool finished = (t_ >= interpolator_.values().back().first) && (!usePostureTransitionCriteria_ || postureTask_->speed().norm() < postureTransitionSpeed_);

  if (repeat_)
  {
    output("Repeat");
  }
  else
  {
    output("OK");
  }
  return finished;
}

void InterpolatePosture::teardown(mc_control::fsm::Controller &ctl)
{
  ctl.gui()->removeCategory({name()});
  if (lookAtActive_)
  {
    ctl.solver().removeTask(lookAt_);
  }
  if (!useDefaultPostureTask_)
  {
    ctl.solver().removeTask(postureTask_);
  }
  if (restorePostureGains_ && useDefaultPostureTask_)
  {
    postureTask_->stiffness(initialPostureStiffness_);
    postureTask_->weight(initialPostureWeight_);
  }
}

EXPORT_SINGLE_STATE("InterpolatePosture", InterpolatePosture)
