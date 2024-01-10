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

  std::vector<std::string> human_devices_; //Devices names on human
  std::string robot_device_; //Device name attached to the robot
  std::string robot_link_; //Robot link on which the device is attached
  sva::PTransformd X_link_sensor_; //Transfo between sensor pose and link pose

  bool human_sim_ = false;
  biRobotTeleop::HumanRobotDataReceiver human_sim_rec_;
  //distant_controller_data
  std::string ip_ = "localhost"; 
  int sub_port_ = 4242;
  int pub_port_ = 4343;
  

};
