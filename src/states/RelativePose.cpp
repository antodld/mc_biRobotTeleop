#include "RelativePose.h"

#include "../BiRobotTeleoperation.h"

void RelativePose::configure(const mc_rtc::Configuration & config)
{
  stateConfig_.load(config);
  stateConfig_("human_indx",h_indx_);
  assert(h_indx_ <= 1);
  stateConfig_("robot_name",r_name_);
  stateConfig_("robot_refFrame",robotRefFrame_);
  stateConfig_("human_robot_RefTransfo",X_RefHuman_RefRobot_);
  stateConfig_("human_robot_TargetTransfo",X_TargetHuman_TargetRobot_);

  if(stateConfig_.has("completion_eval")){stateConfig_("completion_eval",completion_eval_);}

  humanTargetLimb_ = biRobotTeleop::str2Limb(stateConfig_("human_targetLimb"));
}

void RelativePose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const mc_rbdyn::Robot & robot = ctl_.robots().robot(r_name_);
  std::string frame = stateConfig_("task")("frame");
  task_ = std::make_shared<mc_tasks::TransformTask>(robot.frame(frame));
  
  task_->load(ctl_.solver(),stateConfig_("task"));

  task_->target(task_->frame().position());

  ctl_.solver().addTask(task_);
  weight_ = task_->weight();

  createGUI(ctl_);

}

bool RelativePose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);  
  mc_rbdyn::Robot & robot = ctl.robots().robot(r_name_);

  const biRobotTeleop::HumanPose & h = ctl.getHumanPose(h_indx_);

  X_0_hRef_ = h.getOffset(biRobotTeleop::Limbs::Pelvis) * h.getPose(biRobotTeleop::Limbs::Pelvis);
  X_0_hTarget_ = h.getOffset(humanTargetLimb_) * h.getPose(humanTargetLimb_);
  const auto X_hRef_htarget = X_0_hTarget_* X_0_hRef_.inv();
  const sva::MotionVecd v_limb = h.getOffset(humanTargetLimb_) * h.getVel(humanTargetLimb_);
  const sva::MotionVecd v_target = X_TargetHuman_TargetRobot_ * sva::PTransformd(X_0_hTarget_.rotation()) * v_limb ;

  const sva::PTransformd & X_0_RbtRef = robot.frame(robotRefFrame_).position();


  const sva::PTransformd X_0_taskTarget = X_TargetHuman_TargetRobot_ * X_hRef_htarget * X_RefHuman_RefRobot_.inv() * X_0_RbtRef;
  
  task_->target(X_0_taskTarget);
  task_->refVelB(v_target);

  
  if( sva::rotationError(X_0_taskTarget.rotation(),task_->frame().position().rotation()).norm() < completion_eval_)
  {
    return true;
  }
  
  output("OK");
  return true;
}

void RelativePose::addLocalTransfoGUI(mc_control::fsm::Controller & ctl, const std::string & transfo_name, sva::PTransformd & transfo)
{
  auto & gui = *ctl.gui();
  gui.addElement(this, {"States",name(), "Local Transformations",transfo_name},

                 mc_rtc::gui::ArrayInput(
                     "translation [m]", {"x", "y", "z"},
                     [this,&transfo]() -> const Eigen::Vector3d & { return transfo.translation(); },
                     [this,&transfo](const Eigen::Vector3d & t) { transfo.translation() = t; }),
                 mc_rtc::gui::ArrayInput(
                     "rotation [deg]", {"r", "p", "y"},
                     [this,&transfo]() -> Eigen::Vector3d {
                       return mc_rbdyn::rpyFromMat(transfo.rotation()) * 180. / mc_rtc::constants::PI;
                     },
                     [this,&transfo](const Eigen::Vector3d & rpy) {
                       transfo.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.);
                     }));
}

void RelativePose::createGUI(mc_control::fsm::Controller & ctl)
{
  addLocalTransfoGUI(ctl,"reference : human -> robot",X_RefHuman_RefRobot_);
  addLocalTransfoGUI(ctl,"target : robot -> human",X_TargetHuman_TargetRobot_);

  auto & gui = *ctl.gui();
  gui.addElement(this, {"States",name(), "Local Transformations"},
                        mc_rtc::gui::Transform("Robot Ref Frame",
                            [this,&ctl]()-> sva::PTransformd {return ctl.robots().robot(r_name_).frame(robotRefFrame_).position();} ),
                        mc_rtc::gui::Transform("Human Ref Frame",
                            [this,&ctl]()-> sva::PTransformd {return X_RefHuman_RefRobot_ * X_0_hRef_;} ),
                        mc_rtc::gui::Transform("Human Target Frame",
                            [this,&ctl]()-> sva::PTransformd {return X_TargetHuman_TargetRobot_ * X_0_hTarget_;} )
  );


}

void RelativePose::teardownGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  gui.removeCategory({"States",name()});
}

void RelativePose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.solver().removeTask(task_);
  teardownGUI(ctl_);
}

EXPORT_SINGLE_STATE("RelativePose", RelativePose)
