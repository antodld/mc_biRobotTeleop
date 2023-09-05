#include "BiRobotTeleoperation_Task.h"

#include "../BiRobotTeleoperation.h"

void BiRobotTeleoperation_Task::configure(const mc_rtc::Configuration & config)
{
  state_config_.load(config);
  if(state_config_.has("linearWeight"))
  {
    state_config_("linearWeight")("weightRange",weightRange_);
    state_config_("linearWeight")("distanceRange",weightDistanceRange_);
  }
  
  
}

void BiRobotTeleoperation_Task::start(mc_control::fsm::Controller & ctl_)
{

  state_config_("robot_1")("name",r1_name_);
  state_config_("robot_2")("name",r2_name_);
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const unsigned int r2 = ctl.robots().robot(r2_name_).robotIndex();
  const unsigned int r1 = ctl.robots().robot(r1_name_).robotIndex();

  biTask_ = std::make_shared<mc_tasks::biRobotTeleopTask>(ctl.solver(),r1,r2);
  biTask_->load(ctl.solver(),state_config_);
  mc_rbdyn::Robot & human_1 = ctl_.robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = ctl_.robots().robot("human_2");

  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;
  updateHumanPose(human_1,hp_1);
  updateHumanPose(human_2,hp_2);
  biTask_->updateHuman(hp_1,hp_2);
  
  ctl_.datastore().make<std::shared_ptr<mc_tasks::biRobotTeleopTask>>(biTask_->name()+"_task",biTask_);

  ctl_.solver().addTask(biTask_);
}

bool BiRobotTeleoperation_Task::run(mc_control::fsm::Controller & ctl_)
{

  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;

  ctl_.datastore().get<bilateralTeleop::HumanPose>("human_1",hp_1);
  ctl_.datastore().get<bilateralTeleop::HumanPose>("human_2",hp_2);

  biTask_->updateHuman(hp_1,hp_2);

  if(weightDistanceRange_.x() != weightDistanceRange_.y())
  {
    std::vector<sva::PTransformd> localTransfo = biTask_->getRelativeTransfo();
    const double d1 = localTransfo[0].translation().norm();
    const double d2 = localTransfo[1].translation().norm();
    const double d = std::min(d1,d2);
    Eigen::Matrix2d A;
    A << weightDistanceRange_.x() , 1 , weightDistanceRange_.y() ,1;
    const Eigen::Vector2d coeff = A.inverse() * weightRange_; 
    double w = d * coeff.x() + coeff.y();
    w = std::min(std::max(w,weightRange_.x()),weightRange_.y());
    biTask_->weight(w);
  }

  ctl_.datastore().assign<std::shared_ptr<mc_tasks::biRobotTeleopTask>>( biTask_->name() +"_task",biTask_);

  output("OK");
  return false;

}

void BiRobotTeleoperation_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl_.solver().removeTask(biTask_);
  ctl_.datastore().remove(biTask_->name()+"_task");
}

EXPORT_SINGLE_STATE("BiRobotTeleoperation_Task", BiRobotTeleoperation_Task)
