#include "BiRobotTeleoperation.h"

BiRobotTeleoperation::BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

bool BiRobotTeleoperation::run()
{
  return mc_control::fsm::Controller::run();
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


