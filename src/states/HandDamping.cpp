#include "HandDamping.h"

#include "../BiRobotTeleoperation.h"

void HandDamping::configure(const mc_rtc::Configuration & config)
{
  stateConfig_.load(config);
  stateConfig_("human_indx",h_indx_);
  assert(h_indx_ <= 1);
  stateConfig_("robot_name",r_name_);
  humanTargetLimb_ = biRobotTeleop::str2Limb(stateConfig_("human_targetLimb"));


}

void HandDamping::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
 
  const mc_rbdyn::Robot & robot = ctl_.robots().robot(r_name_);
  std::string frame = stateConfig_("task")("frame");
  task_ = std::make_shared<mc_tasks::TransformTask>(robot.frame(frame));


  task_->load(ctl_.solver(),stateConfig_("task"));


  task_->target(task_->frame().position());

  ctl_.solver().addTask(task_);

}

bool HandDamping::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  const biRobotTeleop::HumanPose & h = h_indx_ == 0 ? ctl.hp_1_ : ctl.hp_2_;

  sva::PTransformd X_frame0_frame = sva::PTransformd(task_->frame().position().rotation(),Eigen::Vector3d::Zero());
  task_->refVelB( X_frame0_frame * h.getVel(humanTargetLimb_) );
  
  task_->target(task_->frame().position());

  output("OK");
  return false;
}

void HandDamping::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.solver().removeTask(task_);
}

EXPORT_SINGLE_STATE("HandDamping", HandDamping)
