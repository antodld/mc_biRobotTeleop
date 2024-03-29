#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/DampingTask.h>
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

  /**
   * @brief Compute the human contact limb with robot limb
   * 
   * @param ctl_ 
   * @param robot_indx 
   * @param robot_limb 
   * @return const biRobotTeleop::Limbs 
   */
  const biRobotTeleop::Limbs getContactLimb(mc_control::fsm::Controller & ctl_,const int robot_indx,const biRobotTeleop::Limbs & robot_limb) const;

  /**
   * @brief Get the contact location on the robot indx limb, it suppose contact exist
   * 
   * @param ctl_ 
   * @param robot_indx 
   * @param limb_robot 
   * @param limb_human 
   * @return Eigen::Vector3d 
   */
  const Eigen::Vector3d getContactDistance(mc_control::fsm::Controller & ctl_,const int robot_indx,const biRobotTeleop::Limbs limb_robot,const biRobotTeleop::Limbs limb_human) const;

  bool checkActivation(mc_control::fsm::Controller & ctl_,const int robot_indx);

  double dt_ = 5e-3;
  
  std::shared_ptr<mc_tasks::force::DampingTask> task_a_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_b_;

  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_2_;
  std::shared_ptr<mc_tasks::force::DampingTask> task_robot_1_;

  std::string robot_b_custom_force_sensor_name_;


  int indx_ = 0; //map robot to a either 1 or 2

  biRobotTeleop::Limbs limb_a_ = biRobotTeleop::Limbs::Head; //limbs of task on robot a
  biRobotTeleop::Limbs limb_b_ = biRobotTeleop::Limbs::Head; //limbs of task on robot b

  bool done_ = false;

  bool active_ = false;

  sva::PTransformd X_r1_r2_ = sva::PTransformd::Identity(); //offset transfo from r1 task frame to r2 task frame; 

  std::vector<mc_filter::LowPass<sva::ForceVecd>> activation_force_measurements_robot_1_; //the threshold must be over a low pass filtered value of the force/sensor
  std::vector<mc_filter::LowPass<sva::ForceVecd>> activation_force_measurements_robot_2_; //the threshold must be over a low pass filtered value of the force/sensor
  std::vector<bool> robot_1_force_activation_; //to enforce activation
  std::vector<bool> robot_2_force_activation_; //to enforce activation
  bool activation_enforced_ = false;

  std::string robot_b_name_;
  std::string robot_a_name_;

  mc_filter::LowPass<sva::ForceVecd>* active_force_measurement_  = nullptr; //If the active force control filtered measurement is below the threshold, the force control is deactivated

  std::string contact_limb_; //limb in contact with a robot link equipped of force sensors 
  std::vector<std::string> force_sensor_limbs_robot_1_;
  std::vector<std::string> force_sensor_limbs_robot_2_;


  sva::ForceVecd measured_force_robot_1_ = sva::ForceVecd::Zero(); //measured force in the link frame
  sva::ForceVecd measured_force_robot_2_ = sva::ForceVecd::Zero(); //measured force in the link frame


  double force_activation_threshold_ = 10; //force threshold on which the force control is activated;
  double distance_activation_threshold_ = 0.02;
  double deactivation_threshold_ = 0.15; //distance threshold on which the force control is deactivated

};
