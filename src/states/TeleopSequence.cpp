#include "TeleopSequence.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/io_utils.h>

#include "config.h"

using namespace mc_rtc::gui;

void TeleopSequence::start(mc_control::fsm::Controller & ctl)
{
  const auto mode = ctl.config()("mode", std::string{"None"});
  mc_rtc::log::info("[{}] Starting TeleopSequence in mode {}", name(), mode);

  // Load per-robot config
  if(auto robotConfig = config_.find(ctl.robot().name()))
  {
    config_.load(*robotConfig);
  }

  if(auto modeConfig = config_("Mode", mc_rtc::Configuration{}).find(mode))
  {
    mc_rtc::log::info("[{}] Overriding config for mode {}", name(), mode);
    config_.load(*modeConfig);
  }

  mc_rtc::log::info("[{}] Loaded states: {}", name(),
                    mc_rtc::io::to_string(config_("states", std::vector<std::string>{})));

  ParallelState::start(ctl);
}

bool TeleopSequence::run(mc_control::fsm::Controller & ctl)
{
  return ParallelState::run(ctl);
}

EXPORT_SINGLE_STATE("TeleopSequence", TeleopSequence)
