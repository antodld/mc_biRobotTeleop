#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>
#include <biRobotTeleop/type.h>
#include <biRobotTeleop/HumanRobotPose.h>
#include <mc_filter/LowPass.h>

struct ForceTransmissionLocal : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  const biRobotTeleop::Limbs getContactLimb(mc_control::fsm::Controller & ctl_,const int robot_indx,const std::string & frame) const;

  double getContactLimbDistance(mc_control::fsm::Controller & ctl_,const int robot_indx,const std::string & frame,const biRobotTeleop::Limbs limb) const;

  /**
   * @brief Get the contact location on the robot indx limb, it suppose contact exist
   * 
   * @param ctl_ 
   * @param robot_indx 
   * @param limb_robot 
   * @param limb_human 
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d getContactPose(mc_control::fsm::Controller & ctl_,const int robot_indx,const biRobotTeleop::Limbs limb_robot,const biRobotTeleop::Limbs limb_human);

  bool checkActivation(mc_control::fsm::Controller & ctl_,const int robot_indx);

  double dt_ = 5e-3;
  
  std::shared_ptr<mc_tasks::force::AdmittanceTask> task_a_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> task_b_;

  std::shared_ptr<mc_tasks::force::AdmittanceTask> task_robot_2_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> task_robot_1_;

  int indx_ = 0; //map robot to a either 1 or 2

  biRobotTeleop::Limbs limb_a_ = biRobotTeleop::Limbs::Head; //limbs of task a
  biRobotTeleop::Limbs limb_b_ = biRobotTeleop::Limbs::Head; //limbs of task b

  bool done_ = false;

  bool active_ = false;

  sva::PTransformd X_r1_r2_ = sva::PTransformd::Identity(); //offset transfo from r1 task frame to r2 task frame; 

  std::vector<mc_filter::LowPass<sva::ForceVecd>> activation_force_measurements_robot_1_; //the threshold must be over a low pass filtered value of the force/sensor
  std::vector<mc_filter::LowPass<sva::ForceVecd>> activation_force_measurements_robot_2_; //the threshold must be over a low pass filtered value of the force/sensor
  std::vector<bool> robot_1_force_activation_;
  std::vector<bool> robot_2_force_activation_;

  mc_filter::LowPass<sva::ForceVecd>* active_force_measurement_  = nullptr; //If the active force control filtered measurement is below the threshold, the force control is deactivated

  std::string contact_limb_; //limb in contact with a robot link equipped of force sensors 
  std::vector<std::string> force_sensor_limbs_robot_1_;
  std::vector<std::string> force_sensor_limbs_robot_2_;


  sva::ForceVecd measured_force_robot_1_ = sva::ForceVecd::Zero(); //measured force in the link frame
  sva::ForceVecd measured_force_robot_2_ = sva::ForceVecd::Zero(); //measured force in the link frame


  biRobotTeleop::RobotPose robot_1_pose_;
  biRobotTeleop::RobotPose robot_2_pose_;

  sva::PTransformd X_f_contactF_b = sva::PTransformd::Identity();
  sva::PTransformd X_f_contactF_a = sva::PTransformd::Identity();


  double activation_threshold_ = 10; //force threshold on which the force control is activated;
  double deactivation_threshold_ = 0.15; //distance threshold on which the force control is deactivated

};
