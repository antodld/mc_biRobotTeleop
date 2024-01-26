#pragma once
#include <mc_tasks/PostureTask.h>
#include <mc_control/fsm/State.h>
#include <mc_joystick_plugin/joystick_inputs.h>

namespace mc_control
{
namespace fsm
{
struct Controller;
} // namespace fsm
} // namespace mc_control

struct ChooseTransition : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  bool joystickButtonPressed(mc_control::fsm::Controller & ctl_,const joystickButtonInputs input);

private:
  std::map<joystickButtonInputs,std::string> joystick_actions_; //map joystick input to actions
  std::map<std::string,joystickButtonInputs> actions_joystick_; //map actions to joystick inputs
  std::vector<std::string> gui_action_names_; 

  std::map<std::string, std::string> actions_;
  std::vector<std::string> category_ = {"ChooseTransition"};
  bool fixRobot_ = false; //if true a posture task to fix the robot state will be added
  std::vector<std::shared_ptr<mc_tasks::PostureTask>> tasks_;
  std::vector<std::string> robot_names_;

  mc_rtc::Configuration config_;
};
