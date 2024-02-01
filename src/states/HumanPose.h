#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>
#include "../BiRobotTeleoperation.h"


struct HumanPose : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  bool checkNorm(const sva::PTransformd & X_0_a, const sva::PTransformd & X_0_b = sva::PTransformd::Identity() ,const double tol = 1e-6)
  {
    return (X_0_a.translation() - X_0_b.translation()).norm() < tol;
  }

  void calibrateSensorPose(mc_control::fsm::Controller & ctl, const std::string & robot_link, const std::string & device);

  std::map<std::string,biRobotTeleop::Limbs> human_deviceTolimbs_; //Map linking devices names to human limb attached
  std::string robot_device_; //Device name attached to the robot
  std::string robot_link_; //Robot link on which the device is attached
  sva::PTransformd X_link_sensor_; //Transfo between sensor pose and link pose
  sva::PTransformd offset_global; 

  std::map<biRobotTeleop::Limbs,int> online_data_count_; //One counter for each limbs, reseted each time a data has been updated, set the data as offline if the counter is above a threshold
  int offline_threshold_ = 100;

  bool human_sim_ = false;
  biRobotTeleop::HumanRobotDataReceiver human_sim_rec_;
  //distant_controller_data
  std::string ip_ = "localhost"; 
  int sub_port_ = 4242;
  int pub_port_ = 4343;

  int human_indx_ = 0;
  std::string robot_name_ = "";

  std::string calibration_robot_link_;
  std::string calibration_device_;
  sva::PTransformd link_calib_offset_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_calibTarget_ = sva::PTransformd::Identity();

};
