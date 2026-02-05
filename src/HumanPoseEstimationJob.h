#pragma once

#include <mc_rtc/threading/AsyncJob.h>

#include "BiRobotTeleoperation.h"
#include <mc_rtc_ros/ros.h>

/**
 * Inputs to the async job HumanPoseEstimationJob
 * These should only be copies of existing data, references and pointers are not thread safe in this context
 */
struct HumanPoseEstimationInput
{
  biRobotTeleop::HumanPose h_measured_;

  void init(BiRobotTeleoperation & ctl, mc_rtc::Configuration & config, const std::string & name)
  {

    h_measured_ = biRobotTeleop::HumanPose(name);
  }

  void syncState(BiRobotTeleoperation & ctl, int human_indx_)
  {
    ctl.updateHumanPose(ctl.getHumanPose(human_indx_), h_measured_);
    h_measured_.setOffset(ctl.getHumanPose(human_indx_).getOffset());
  }
};

/**
 * Copies of the async job outputs
 */
struct HumanPoseEstimationResult
{
  biRobotTeleop::HumanPose h_estimated_;

  void init(const std::string & name)
  {
    h_estimated_ = biRobotTeleop::HumanPose(name);
  }

  void syncResult(BiRobotTeleoperation & ctl, int human_indx)
  {
    ctl.updateHumanPose(h_estimated_, ctl.getHumanPose(human_indx, true));
  }

  /// this ref should only be used to copy the result state into an actual robot, it should not be stored
  /// and used while an async job is running
  const mc_rbdyn::Robot & robot() const
  {
    return ext_robots->robot();
  }

protected:
  mc_rbdyn::RobotsPtr
      ext_robots; // holds an additional human robot instance used by HumanPoseEstimationJob::computeJob async task
};

/**
 * Perform pose estimation of a human pose as an async job
 *
 * Inputs: HumanPose data from OpenVR trackers
 * Results: Estimated HumanPose data + a corresponding human robot instance. Caution this robot should not be used
 * directly in the controller, it is only thread-safe to copy it when no async jobs are running
 *
 * computeJob performs the actual estimation using a least-mean square optimization
 */
struct HumanPoseEstimationJob
: public mc_rtc::threading::MakeAsyncJob<HumanPoseEstimationJob, HumanPoseEstimationInput, HumanPoseEstimationResult>
{
  // Read-only members
  std::string humanRobot_name_;
  int human_indx_ = 0;
  std::vector<biRobotTeleop::Limbs> target_limbs_;
  double stiffness_ = 100;
  double dt_ = 0.05;
  biRobotTeleop::RobotPose humanRobot_links_;
  double solving_perf_ = 0;
  mc_rbdyn::RobotsPtr ext_robots;
  std::string name_;

  HumanPoseEstimationJob(BiRobotTeleoperation & ctl,
                         const mc_rbdyn::Robots & ext_robots,
                         const std::string & humanRobotName,
                         const mc_rtc::Configuration & config,
                         const std::string & name);

  /**
   * Update the estimator's robot model using the RobotModelUpdate plugin
   * You are expected to manually call this function when the corresponding display robot scale changes in the plugin
   * (see HumanPoseEstimation)
   *
   * \WARNING:
   * - this function should never be called while an async job is running as this would not be thread-safe
   */
  void updateRobotModelScale(mc_control::fsm::Controller & ctl)
  {
    if(ctl.datastore().has("RobotModelUpdate_" + humanRobot_name_ + "::updateRobotModel"))
    {
      ctl.datastore().call("RobotModelUpdate_" + humanRobot_name_ + "::updateRobotModel", ext_robots->robot());
    }
  }

  // Async job performing the actual estimation
  HumanPoseEstimationResult computeJob();

  // Deferred CRTP logger implementation
  void addToLoggerImpl()
  {
    auto prefix = "perf_" + loggerPrefix_ + "_async_";
    logger_->addLogEntry(prefix + "solve [ms]", this, [this]() { return dt_solve_.load(); });
  }

  void addToGUIImpl() {}

protected:
  HumanPoseEstimationResult result;
  std::atomic<double> dt_solve_ = 0.;

  // each task is in the form min(A * \ddot{q} - b)
  std::vector<Eigen::MatrixXd> task_mat_;
  std::vector<Eigen::VectorXd> task_vec_;
  std::vector<double> task_weight_;
  Eigen::VectorXd dot_q_;

protected: // least-mean square implementation
  void addTransformTask(mc_rbdyn::Robot & human,
                        const std::string & human_link,
                        const sva::PTransformd & X_0_target,
                        const sva::MotionVecd & targetVel,
                        const double stiffness = 100.,
                        const double weight = 10.);

  void addPostureTask(mc_rbdyn::Robot & human, const double stiffness = 1., const double weight = 1.);

  void addMinAccTask(mc_rbdyn::Robot & human, const double weight = 1.);

  Eigen::VectorXd solve();

  void set_estimated_values(mc_rbdyn::Robot & human,
                            biRobotTeleop::HumanPose & h_estimated_,
                            const std::string & link,
                            biRobotTeleop::Limbs limb);
};
