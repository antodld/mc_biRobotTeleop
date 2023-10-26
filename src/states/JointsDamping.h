#pragma once

#include <mc_control/fsm/State.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/biRobotTeleopTask.h>
#include <biRobotTeleop/type.h>

struct JointsDamping : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  std::string r_name_ = "";
  unsigned int h_indx_ = 0;
  unsigned int r_indx_ = 0;

  mc_rtc::Configuration stateConfig_;

  std::shared_ptr<mc_tasks::PostureTask> task_;

  std::vector<std::string> joints_;
  biRobotTeleop::Limbs measured_limb_;
  double damping_ = 100;


};
