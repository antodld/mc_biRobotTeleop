#include "BiRobotTeleoperation_Initial.h"

#include "../BiRobotTeleoperation.h"

void BiRobotTeleoperation_Initial::configure(const mc_rtc::Configuration & config)
{
}

void BiRobotTeleoperation_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
}

bool BiRobotTeleoperation_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  output("OK");
  return true;
}

void BiRobotTeleoperation_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
}

EXPORT_SINGLE_STATE("BiRobotTeleoperation_Initial", BiRobotTeleoperation_Initial)
