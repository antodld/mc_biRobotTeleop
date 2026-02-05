#pragma once
#include <mc_control/fsm/states/Parallel.h>

namespace mc_control
{
namespace fsm
{
struct Controller;
} // namespace fsm
} // namespace mc_control

struct TeleopSequence : mc_control::fsm::ParallelState
{
  enum class Mode
  {
    None,
    SimulationSingle,
    SingleVR,
    DualVR
  };
  inline std::string to_string(Mode mode)
  {
    switch(mode)
    {
      case Mode::None:
        return "None";
      case Mode::SimulationSingle:
        return "SimulationSingle";
      case Mode::SingleVR:
        return "SingleVR";
      case Mode::DualVR:
        return "DualVR";
      default:
        return "Unknown";
    }
  }

  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
};
