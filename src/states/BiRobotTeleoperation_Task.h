#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/biRobotTeleopTask.h>
#include <bilateralteleop/HumanRobotPose.h>
#include <bilateralTeleop_dataLink/type.h>

struct BiRobotTeleoperation_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addToLogger(mc_control::fsm::Controller & ctl);

  void teardownLogger(mc_control::fsm::Controller & ctl);

  std::shared_ptr<mc_tasks::biRobotTeleopTask> biTask_;

  std::vector<std::shared_ptr<mc_tasks::biRobotTeleopTask>> biTasks_;

  std::vector<bilateralTeleop::Limbs> r1_links_ = {};
  std::vector<bilateralTeleop::Limbs> r2_links_ = {};

  std::string r1_name_ = "";
  std::string r2_name_ = "";

  mc_rtc::Configuration state_config_;

  double weight_ = 10;
  double minDist_ = 0;
  double softMaxGain_ = 10;
  double deltaDistGain_ = 1;
  std::vector<double> dist_;
  //Used to get the transfo for the softmax computation
  size_t indx_sotfM = 0;

  //The weight is a linear interpolation clamped in the distance range between the weight range
  Eigen::Vector2d weightRange_ = {20,1000};
  Eigen::Vector2d weightDistanceRange_ = Eigen::Vector2d::Zero();

};

double softMax(const std::vector<double> & x, const double k , const size_t indx)
{
  double d = 0;
  for (const auto & x_i : x)
  {
    d += exp(k*x_i);
  }
  return  exp(k*x[indx])/d;
}