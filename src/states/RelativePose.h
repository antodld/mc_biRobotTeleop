#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/biRobotTeleopTask.h>

struct RelativePose : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void createGUI(mc_control::fsm::Controller & ctl);

  void teardownGUI(mc_control::fsm::Controller & ctl);

  void addLocalTransfoGUI(mc_control::fsm::Controller & ctl,const std::string & name, sva::PTransformd & transfo);

  std::string r_name_ = "";

  mc_rtc::Configuration stateConfig_;
  
  unsigned int h_indx_ = 0;

  std::string robotRefFrame_ = "";
  sva::PTransformd X_RefHuman_RefRobot_ = sva::PTransformd::Identity();
  sva::PTransformd X_TargetHuman_TargetRobot_ = sva::PTransformd::Identity();

  sva::PTransformd X_0_hRef_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_hTarget_ = sva::PTransformd::Identity();

  biRobotTeleop::Limbs humanTargetLimb_;
  std::shared_ptr<mc_tasks::TransformTask> task_;
  double completion_eval_ = 0;


};
