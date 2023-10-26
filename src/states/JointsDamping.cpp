#include "JointsDamping.h"

#include "../BiRobotTeleoperation.h"

void JointsDamping::configure(const mc_rtc::Configuration & config)
{
  stateConfig_.load(config);
  stateConfig_("human_indx",h_indx_);
  assert(h_indx_ <= 1);
  stateConfig_("robot_name",r_name_);
  stateConfig_("joints",joints_);
  std::string limb;
  stateConfig_("measured_limb",limb);
  stateConfig_("damping",damping_);
  measured_limb_ = biRobotTeleop::str2Limb(limb);


}

void JointsDamping::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
 
  const mc_rbdyn::Robot & robot = ctl_.robots().robot(r_name_);

  task_ = std::make_shared<mc_tasks::PostureTask>(ctl.solver(),robot.robotIndex());

  if(stateConfig_.has("task"))
  {
    task_->load(ctl_.solver(),stateConfig_("task"));
  }

  task_->stiffness(0);
  task_->selectActiveJoints(ctl.solver(),joints_);

  ctl_.solver().addTask(task_);

}

bool JointsDamping::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  biRobotTeleop::HumanPose h;

  if(h_indx_ == 0)
  {
    ctl_.datastore().get<biRobotTeleop::HumanPose>("human_1",h);
  }
  else
  {
    ctl_.datastore().get<biRobotTeleop::HumanPose>("human_2",h);
  }

  const sva::MotionVecd vel = h.getVel(measured_limb_);

  if(vel.linear().norm() < 1e-2)
  {
    task_->damping(damping_);
  }
  else
  {
    task_->damping(0);
  }


  output("OK");
  return false;
}

void JointsDamping::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.solver().removeTask(task_);
}

EXPORT_SINGLE_STATE("JointsDamping", JointsDamping)
