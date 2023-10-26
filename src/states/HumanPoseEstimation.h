#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>

struct HumanPoseEstimation : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  void addTransformTask(mc_rbdyn::Robot & human,const std::string & human_link, const sva::PTransformd & X_0_target, const sva::MotionVecd & targetVel,
                        const double stffness = 100., const double weight = 10.);

  void addPostureTask(mc_rbdyn::Robot & human,const double stiffness = 1., const double weight = 1.);

  void addMinAccTask(mc_rbdyn::Robot & human,const double weight = 1.);

  Eigen::VectorXd solve();

  int human_indx_ = 0;

  int robot_indx_ = 0;

  double dt_ = 0.05;
  std::string humanRobot_name_;

    //each task is in the form min(A * \ddot{q} - b)
  std::vector<Eigen::MatrixXd> task_mat_;
  std::vector<Eigen::VectorXd> task_vec_;
  std::vector<double> task_weight_;
  Eigen::VectorXd dot_q_;

};
