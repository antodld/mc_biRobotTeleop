#include "ChooseTransition.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/Button.h>
#include <mc_joystick_plugin/joystick_inputs.h>


void ChooseTransition::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);

}

void ChooseTransition::start(mc_control::fsm::Controller & ctl_)
{
  using namespace mc_rtc::gui;

  config_("category", category_);
  config_("actions", actions_);
  config_("fix_robot",fixRobot_);
  std::vector<std::vector<std::string>> joystick_actions;
  if(config_.has("joystick_action"))
  {
    config_("joystick_action",joystick_actions);
    for (auto & pair : joystick_actions)
    {
      joystick_actions_[strToButtons(pair[0])] = pair[1];
      actions_joystick_[pair[1]] = strToButtons(pair[0]);
    }
  }
  mc_rtc::log::info("[{}] State config {}",name(),config_.dump(true,true));

  for(const auto & action : actions_)
  {
    std::string action_name = action.first;
    if(actions_joystick_.find(action.second) != actions_joystick_.end())
    {
      action_name += " : " + buttons2str(actions_joystick_[action.second]);
    }
    mc_rtc::log::info("[{}] Adding button: {}",name(),action_name);
    ctl_.gui()->addElement(category_, Button(action_name, [this, action]() {
                            mc_rtc::log::info("[{}] Action {} chosen, triggering output {}", name(), action.first,
                                              action.second);
                            output(action.second);
                          }));
    gui_action_names_.push_back(action_name);
  }

  if(fixRobot_)
  {
    config_("robots",robot_names_);
    for (auto & r_name : robot_names_)
    {
      auto r_indx = ctl_.robots().robotIndex(r_name);
      tasks_.push_back(std::make_shared<mc_tasks::PostureTask>(ctl_.solver(),r_indx));
      if(config_.has("task"))
      {
        tasks_.back()->load(ctl_.solver(),config_("task"));
        tasks_.back()->name(name() + "_posture_" + r_name);
      }
      ctl_.solver().addTask(tasks_.back());
    }
  }

}

bool ChooseTransition::run(mc_control::fsm::Controller & ctl_)
{

  for(auto & joystick_in : joystick_actions_)
  {
    if(joystickButtonPressed(ctl_,joystick_in.first))
    {
      output(joystick_in.second);
      mc_rtc::log::info("[{}] Action on joystick chosen, triggering output {}", name(),joystick_in.second);
    }
  }
  if(output().empty())
  {
    return false;
  }
  return true;
}

bool ChooseTransition::joystickButtonPressed(mc_control::fsm::Controller & ctl_,const joystickButtonInputs input)
{
  bool joystick_online = false;

  if(ctl_.datastore().has("Joystick::connected"))
  {
    joystick_online = ctl_.datastore().get<bool>("Joystick::connected");
  }

  if(joystick_online)
  {
    auto & buttonEvent_func = ctl_.datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::ButtonEvent");
    auto & button_func = ctl_.datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::Button");
    return  button_func(input) && buttonEvent_func(input);
  }
  return false;
}

void ChooseTransition::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & action : gui_action_names_)
  {
    ctl.gui()->removeElement(category_, action);
  }
  if(fixRobot_)
  {
    for (auto & task : tasks_)
    {
      ctl.solver().removeTask(task);
    }
  }
  actions_.clear();
  joystick_actions_.clear();
  actions_joystick_.clear();
}

EXPORT_SINGLE_STATE("ChooseTransition", ChooseTransition)
