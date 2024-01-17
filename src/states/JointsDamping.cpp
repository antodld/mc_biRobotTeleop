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

  vel_filter_ = mc_filter::LowPass<sva::MotionVecd>(ctl.timeStep,0.3);
  vel_filter_.reset(sva::MotionVecd::Zero());

  task_ = std::make_shared<mc_tasks::PostureTask>(ctl.solver(),robot.robotIndex());

  task_->name(name() + "_task");

  if(stateConfig_.has("task"))
  {
    task_->load(ctl_.solver(),stateConfig_("task"));
  }

  task_->stiffness(0);
  task_->selectActiveJoints(ctl.solver(),joints_);

  ctl_.solver().addTask(task_);

  ctl.logger().addLogEntry(name() + "_velocity",[this]() -> const sva::MotionVecd & {return vel_filter_.eval();});
  ctl.logger().addLogEntry(name() + "damping active",[this]() -> const bool & {return damping_on_;});


}

bool JointsDamping::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  const biRobotTeleop::HumanPose & h = ctl.getHumanPose(h_indx_);

  const sva::MotionVecd vel = h.getVel(measured_limb_);

  vel_filter_.update(vel);

  if(vel_filter_.eval().vector().norm() < 1e-2 && !damping_on_)
  {
    task_->damping(damping_);
    damping_on_ = true;
  }
  else if (vel.vector().norm() >= 1e-2 && damping_on_)
  {
    vel_filter_.reset(vel);
    task_->damping(0);
    damping_on_ = false;
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
