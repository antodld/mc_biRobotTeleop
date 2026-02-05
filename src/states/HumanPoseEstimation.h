#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Input.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/TransformTask.h>

#include "../HumanPoseEstimationJob.h"

struct HumanPoseEstimation : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  std::unique_ptr<HumanPoseEstimationJob> job_;
  std::unique_ptr<mc_rtc::RobotPublisher> rpub_;
  bool humanScaleUpdated_ = false; // true when the RobotModelUpdate plugin has modified the human scale
};
