#include "BiRobotTeleoperation_Initial.h"

#include "../BiRobotTeleoperation.h"

void BiRobotTeleoperation_Initial::configure(const mc_rtc::Configuration & config)
{
}

void BiRobotTeleoperation_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.datastore().make<bool>("robot_1_IsReady", false);
  run(ctl_);

}

bool BiRobotTeleoperation_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.CalibrateExtWrench(ctl.realRobot("robot_1"));
  ctl.CalibrateExtWrench(ctl.realRobot("robot_2"));
  count_++;
  output("OK");
  
  if(ctl.datastore().has("UDPPlugin"))
  { // When using the UDP plugin wait until hrp4 is ready
    const std::string datastore_func_name = "UDPPlugin::robot_1::reset";
    if(!ctl.datastore().has(datastore_func_name))
    {
      ctl.datastore().make_call(datastore_func_name,
                            [this,&ctl]()
                            {
                              mc_rtc::log::warning("TestUDPPugin::reset called, resetting posture task");
                              ctl.datastore().assign<bool>("robot_1_IsReady", true);
                              ctl.getPostureTask("robot_1")->reset();
                            });
      mc_rtc::log::info("waiting for robot_1 on udp");

    }
    return ctl.datastore().get<bool>("robot_1_IsReady") && count_ < 20;
  }

  return count_ < 20;
  
}

void BiRobotTeleoperation_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
}

EXPORT_SINGLE_STATE("BiRobotTeleoperation_Initial", BiRobotTeleoperation_Initial)
