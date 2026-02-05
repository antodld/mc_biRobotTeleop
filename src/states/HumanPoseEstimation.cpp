#include "HumanPoseEstimation.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Robot.h>

#include <biRobotTeleop/HumanRobotPose.h>
#include <mc_joystick_plugin/joystick_inputs.h>

void HumanPoseEstimation::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  const auto humanRobot_name = "human_" + std::to_string(config_("human_indx", 0) + 1) + "_estimated";

  // Add an external robot to display the output
  auto rm = mc_rbdyn::RobotLoader::get_robot_module(ctl.config()("estimation_module", std::string{"simple_human"}));
  // auto rm = mc_rbdyn::RobotLoader::get_robot_module("human");
  auto & human = ctl.external_robots_->load(humanRobot_name, *rm);
  mc_rtc::log::info("[{}] Loaded external robot \"{}\"", name(), human.name());

  if(ctl.datastore().has("RobotModelUpdate_" + humanRobot_name + "::registerRobot"))
  {
    /**
     * \NOTE: there is a bit of wizardry here:
     * - We register the display "human" robot handled by this (HumanPoseEstimation) in the RobotModelUpdate plugin
     *   This is fine as this robot is in the controller thread
     * - To update the estimator's corresponding robot (HumanPoseEstimationJob), we listen to updates to the registed
     * "human" robot from the plugin, and manually trigger a call to RobotModelUpdate::updateRobotModel on the
     * estimator's robot when suitable (before running the next async job)
     *  - This makes updates to the estimator's robot thread-safe
     */
    ctl.datastore().call("RobotModelUpdate_" + humanRobot_name + "::registerRobot", human,
                         std::function<void()>([this]() { humanScaleUpdated_ = true; }));
  }

  job_ = std::make_unique<HumanPoseEstimationJob>(ctl, *ctl.external_robots_, humanRobot_name, config_, name());
  if(!job_->startedOnce())
  {
    auto & input = job_->input();
    input.init(ctl, config_, name());
  }

  // Initialize ros publisher
  mc_rtc::log::info("init robot publisher for {}", "control/" + humanRobot_name);

  rpub_ = std::make_unique<mc_rtc::RobotPublisher>("control/" + humanRobot_name + "/",
                                                   mc_rtc::ROSBridge::get_publisher_timestep(), ctl.timeStep);
  rpub_->init(human, false);

  ctl.getGUIBuilder().addElement({"BiRobotTeleop", "Estimated Human"},
                                 mc_rtc::gui::Robot(humanRobot_name,
                                                    [this, &ctl, humanRobot_name]() -> mc_rbdyn::Robot &
                                                    { return ctl.external_robots_->robot(humanRobot_name); }));

  ctl.gui()->addElement({"States", name()},
                        mc_rtc::gui::Robot(humanRobot_name, [this, &ctl, humanRobot_name]() -> const mc_rbdyn::Robot &
                                           { return ctl.external_robots_->robot(humanRobot_name); }));

  run(ctl_);
}

bool HumanPoseEstimation::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  if(!job_->running())
  {
    // initialize input when no job is running
    auto & input = job_->input();
    input.syncState(ctl, job_->human_indx_);

    if(humanScaleUpdated_)
    {
      // trigger update to the corersponding estimator's robot when the RobotModelUpdate plugin has updated it
      job_->updateRobotModelScale(ctl_);
      humanScaleUpdated_ = false;
    }

    // Job starts as an async task, use job.checkResult() later to know whether it is finished and retrive its value
    job_->startAsync();
    job_->addToLogger(ctl_.logger(), name());
    // job_->addToGUI(*ctl_.gui(), guiCategory_);
  }
  else if(job_->checkResult())
  {
    const auto & result = *job_->lastResult();
    ctl.updateHumanPose(result.h_estimated_, ctl.getHumanPose(job_->human_indx_, true));

    // copy human robot state for display and publishing to external controller
    auto & threadRobot = job_->ext_robots->robot(job_->humanRobot_name_);
    auto & human = ctl.external_robots_->robot(job_->humanRobot_name_);
    human.mbc() = threadRobot.mbc();
    human.forwardKinematics();
    human.forwardVelocity();
    human.forwardAcceleration();
    // mc_rtc::log::info("Update robot publisher \"control/{}\"", job_->humanRobot_name_);
    rpub_->update(ctl.timeStep, human);
  }

  output("True");
  return false;
}

// void HumanPoseEstimation::addLog(mc_control::fsm::Controller & ctl_)
// {
//   ctl_.logger().addLogEntry("perf_" + name() + "_solver", [this]() -> const double { return solving_perf_; });
// }
//
// void HumanPoseEstimation::addGUI(mc_control::fsm::Controller & ctl_) {}

void HumanPoseEstimation::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  auto & robots = ctl.external_robots_;
  auto & human = robots->robot(job_->humanRobot_name_);
  if(ctl.datastore().has("RobotModelUpdate_" + job_->humanRobot_name_ + "::unregisterRobot"))
  {
    ctl.datastore().call("RobotModelUpdate_" + job_->humanRobot_name_ + "::unregisterRobot", human);
  }
  const int indx = robots->robotIndex(job_->humanRobot_name_);
  robots->removeRobot(indx);
}

EXPORT_SINGLE_STATE("HumanPoseEstimation", HumanPoseEstimation)
