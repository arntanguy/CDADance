#pragma once

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>
#include <memory>
#include <optional>
#include <string>

/** Virtual interface to the walking controller */
struct WalkingInterface
{
  /** Returns true if the robot is walking */
  virtual bool is_walking() = 0;

  /** Returns true if the robot is in double support */
  virtual bool is_double_support() = 0;

  /** Returns true if the walking is stopped */
  virtual bool is_stopped() = 0;

  /** Returns tue if a stop has been initiated */
  virtual bool is_stopping() = 0;

  virtual void start_walking() = 0;
  virtual void stop_walking() = 0;

  /** Starts the walk if stopped, stops it otherwise */
  virtual void start_stop_walking() = 0;

  /** Set the reference velocity for walking */
  virtual void set_planner_ref_vel(const Eigen::Vector3d & v) = 0;

  /** Load a predefined plan (LIPMWalking only for now) */
  virtual void load_plan(const std::string & name,
                         const std::optional<sva::PTransformd> & relTarget = std::nullopt) = 0;

  /** Set the desired torso pitch */
  virtual void set_torso_pitch(double p) = 0;

  /** Get the walking CoM height */
  virtual double get_com_height() = 0;

  /** Set the walking CoM height */
  virtual void set_com_height(double h) = 0;

  /** Get the current support foot */
  virtual std::string get_support_foot() = 0;

  /** Get current ZMP target */
  virtual Eigen::Vector3d get_zmp_target() = 0;

  /** Disable stabilizer task (remove from QP) */
  virtual void remove_stabilizer_task() = 0;
};

using WalkingInterfacePtr = std::shared_ptr<WalkingInterface>;
