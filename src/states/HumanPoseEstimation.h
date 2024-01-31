#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>
#include "../BiRobotTeleoperation.h"


struct HumanPoseEstimation : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void runThread(mc_control::fsm::Controller & ctl_);

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  void addTransformTask(mc_rbdyn::Robot & human,const std::string & human_link, const sva::PTransformd & X_0_target, const sva::MotionVecd & targetVel,
                        const double stffness = 100., const double weight = 10.);

  void addPostureTask(mc_rbdyn::Robot & human,const double stiffness = 1., const double weight = 1.);

  void addMinAccTask(mc_rbdyn::Robot & human,const double weight = 1.);

  void set_estimated_values(mc_rbdyn::Robot & human, const std::string & link, biRobotTeleop::Limbs limb)
  {
    const sva::PTransformd offset = h_estimated_.getOffset(limb);

    h_estimated_.setPose(limb, offset.inv() * human.frame(link).position());
    h_estimated_.setVel(limb, offset.inv() * human.bodyVelW(link));
    const sva::MotionVecd v = h_estimated_.getVel(limb);
    const sva::MotionVecd & acc_body = human.bodyAccB(link);
    h_estimated_.setAcc(limb, 
                        (offset.inv() *
                        sva::PTransformd(human.frame(link).position().rotation().transpose(),Eigen::Vector3d::Zero()) *
                        acc_body) +
                        sva::MotionVecd(Eigen::Vector3d::Zero(),v.angular().cross(v.linear())));
  }

  Eigen::VectorXd solve();

  int human_indx_ = 0;

  int robot_indx_ = 0;

  double stiffness_ = 100;

  double dt_ = 0.05;
  std::string humanRobot_name_;

    //each task is in the form min(A * \ddot{q} - b)
  std::vector<Eigen::MatrixXd> task_mat_;
  std::vector<Eigen::VectorXd> task_vec_;
  std::vector<double> task_weight_;
  Eigen::VectorXd dot_q_;

  double solving_perf_ = 0;

  bool run_ = true;
  std::thread estimation_thread_;
  biRobotTeleop::HumanPose h_measured_;
  biRobotTeleop::HumanPose h_measured_thread_;
  biRobotTeleop::HumanPose h_estimated_;
  std::vector<biRobotTeleop::Limbs> target_limbs_;
  biRobotTeleop::RobotPose humanRobot_links_;
  std::mutex mutex_copy_;



};
