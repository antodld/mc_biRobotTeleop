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
  // if(state_config_.has("weight"))
  // {
  //   state_config_("weight",weight_);
  // }
  if(state_config_.has("softMaxGain"))
  {
    state_config_("softMaxGain",softMaxGain_);
  }
  
}

void BiRobotTeleoperation_Task::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  mc_rbdyn::Robot & human_1 = ctl_.robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = ctl_.robots().robot("human_2");

  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;
  updateHumanPose(human_1,hp_1);
  updateHumanPose(human_2,hp_2);
  std::vector<std::string> r1_linksName = {};
  std::vector<std::string> r2_linksName = {};

  state_config_("robot_1")("name",r1_name_);
  state_config_("robot_2")("name",r2_name_);
  state_config_("robot_1")("links",r1_linksName);
  state_config_("robot_2")("links",r2_linksName);

  if(r1_linksName.size() == 0 || r2_linksName.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Links list must not be empty {} {}",name(),r1_linksName.size(),r2_linksName.size());
  }
  if(r1_linksName.size() != 1 && r2_linksName.size() != 1)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] One of the link list must contain only one link",name());
  }
  if (r1_linksName.size() <= r2_linksName.size())
  {
    bilateralTeleop::Limbs limb = bilateralTeleop::str2Limb(r1_linksName[0]);
    for(auto & link : r2_linksName)
    {
      r1_links_.push_back(limb);
      std::cout << link << std::endl;
      r2_links_.push_back(bilateralTeleop::str2Limb(link));
    }
  }
  else
  {
    indx_sotfM = 1;
    bilateralTeleop::Limbs limb = bilateralTeleop::str2Limb(r2_linksName[0]);
    for(auto & link : r1_linksName)
    {
      r2_links_.push_back(limb);
      r1_links_.push_back(bilateralTeleop::str2Limb(link));

    }
  }

  const unsigned int r2 = ctl.robots().robot(r2_name_).robotIndex();
  const unsigned int r1 = ctl.robots().robot(r1_name_).robotIndex();

  for (size_t k = 0; k < r1_links_.size() ; k++)
  {
    std::shared_ptr<mc_tasks::biRobotTeleopTask> task = std::make_shared<mc_tasks::biRobotTeleopTask>(ctl.solver(),r1,r2,r1_links_[k],r2_links_[k]);
    task->load(ctl.solver(),state_config_);
    auto name = task->name();
    task->name(name + "_" + std::to_string(k));
    task->updateHuman(hp_1,hp_2);
    ctl_.solver().addTask(task);
    biTasks_.push_back(task);

  }

  addToLogger(ctl_);

  // biTask_ = std::make_shared<mc_tasks::biRobotTeleopTask>(ctl.solver(),r1,r2);
  // biTask_->load(ctl.solver(),state_config_);
  // biTask_->updateHuman(hp_1,hp_2);

  // ctl_.datastore().make<std::shared_ptr<mc_tasks::biRobotTeleopTask>>(biTask_->name()+"_task",biTask_);

  // ctl_.solver().addTask(biTask_);
}

bool BiRobotTeleoperation_Task::run(mc_control::fsm::Controller & ctl_)
{



  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;

  ctl_.datastore().get<bilateralTeleop::HumanPose>("human_1",hp_1);
  ctl_.datastore().get<bilateralTeleop::HumanPose>("human_2",hp_2);

  std::vector<double> dist;
  minDist_ = 1e9;
  for(auto & task : biTasks_)
  {
    task->updateHuman(hp_1,hp_2);
    dist.push_back(task->getRelativeTransfo()[indx_sotfM].translation().norm());
    minDist_ = std::min(minDist_ , dist.back()); 
  }

  if(weightDistanceRange_.x() != weightDistanceRange_.y())
  {

    Eigen::Matrix2d A;
    A << weightDistanceRange_.x() , 1 , weightDistanceRange_.y() ,1;
    const Eigen::Vector2d coeff = A.inverse() * weightRange_; 
    double w = minDist_ * coeff.x() + coeff.y();
    weight_ = std::min(std::max(w,weightRange_.x()),weightRange_.y());
  }

  for(size_t i = 0 ; i < biTasks_.size() ; i++)
  {
    double w = softMax(dist,-softMaxGain_,i);
    biTasks_[i]->weight(weight_ * w);
  }

  // biTask_->updateHuman(hp_1,hp_2);



  // ctl_.datastore().assign<std::shared_ptr<mc_tasks::biRobotTeleopTask>>( biTask_->name() +"_task",biTask_);

  output("OK");
  return false;

}

void BiRobotTeleoperation_Task::addToLogger(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

  logger.addLogEntry( name() + "_reference_weight",[this]() -> const double {return weight_;});
  logger.addLogEntry( name() + "_reference_distance",[this]() -> const double {return minDist_;});
  logger.addLogEntry( name() + "_soft_max_gain",[this]() -> const double {return softMaxGain_;});

}

void BiRobotTeleoperation_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  for (auto & task : biTasks_)
  {
    ctl_.solver().removeTask(task);
  }
  // ctl_.solver().removeTask(biTask_);
  // ctl_.datastore().remove(biTask_->name()+"_task");
}

EXPORT_SINGLE_STATE("BiRobotTeleoperation_Task", BiRobotTeleoperation_Task)
