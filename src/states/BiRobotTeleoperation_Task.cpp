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
  if(state_config_.has("deltaDistGain"))
  {
    state_config_("deltaDistGain",deltaDistGain_);
  }
}

void BiRobotTeleoperation_Task::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  std::vector<std::string> r1_linksName = {};
  std::vector<std::string> r2_linksName = {};

  state_config_("robot_1")("name",r1_name_);
  state_config_("robot_2")("name",r2_name_);
  state_config_("robot_1")("links",r1_linksName);
  state_config_("robot_2")("links",r2_linksName);

  biRobotTeleop::HumanPose hp_1;
  biRobotTeleop::HumanPose hp_2;

  ctl_.datastore().get<biRobotTeleop::HumanPose>("human_1",hp_1);
  ctl_.datastore().get<biRobotTeleop::HumanPose>("human_2",hp_2);

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
    biRobotTeleop::Limbs limb = biRobotTeleop::str2Limb(r1_linksName[0]);
    for(auto & link : r2_linksName)
    {
      r1_links_.push_back(limb);
      std::cout << link << std::endl;
      r2_links_.push_back(biRobotTeleop::str2Limb(link));
    }
  }
  else
  {
    indx_sotfM = 1;
    biRobotTeleop::Limbs limb = biRobotTeleop::str2Limb(r2_linksName[0]);
    for(auto & link : r1_linksName)
    {
      r2_links_.push_back(limb);
      r1_links_.push_back(biRobotTeleop::str2Limb(link));

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
  mc_rtc::log::info("[{}] create datastore {}",name(),name()+"_tasks" );
  ctl_.datastore().make<std::vector<std::shared_ptr<mc_tasks::biRobotTeleopTask>>>( name()+"_tasks",biTasks_);

  // ctl_.solver().addTask(biTask_);

  auto posture_1 = ctl_.getPostureTask(r1_name_);
  auto posture_2 = ctl_.getPostureTask(r2_name_);
  // posture_1->weight(200);
  // posture_2->weight(200);
  // posture_1->stiffness(0);
  // posture_2->stiffness(0);
  // posture_1->damping(300);
  // posture_2->damping(3);

}

bool BiRobotTeleoperation_Task::run(mc_control::fsm::Controller & ctl_)
{



  biRobotTeleop::HumanPose hp_1;
  biRobotTeleop::HumanPose hp_2;

  ctl_.datastore().get<biRobotTeleop::HumanPose>("human_1",hp_1);
  ctl_.datastore().get<biRobotTeleop::HumanPose>("human_2",hp_2);

  auto posture_1 = ctl_.getPostureTask(r1_name_);
  auto posture_2 = ctl_.getPostureTask(r2_name_);

  // if(hp_1.getVel(biRobotTeleop::Limbs::LeftHand).linear().norm() < 0.05)
  // {
  //   posture_1->damping(100);
  // }
  // else
  // {
  //   posture_1->damping(3);
  // }
  // if(hp_2.getVel(biRobotTeleop::Limbs::RightHand).linear().norm() < 0.05)
  // {
  //   posture_2->damping(100);
  // }
  // else
  // {
  //   posture_2->damping(3);
  // }

  std::vector<double> dist;
  std::vector<double> distVel;
  std::vector<double> softMaxVec;
  size_t indx = 0;
  minDist_ = 1e9;
  for(auto & task : biTasks_)
  {
    task->updateHuman(hp_1,hp_2);
    dist.push_back(task->getRelativeTransfo()[indx_sotfM].translation().norm());
    minDist_ = std::min(minDist_ , dist.back()); 
    if(dist_.size() > indx){distVel.push_back( dist.back() - dist_[indx]);}
    else{distVel.push_back(0.);}
    softMaxVec.push_back(dist.back() + deltaDistGain_ * exp(distVel.back()) );
    indx++;
  }
  dist_ = dist;

  if(weightDistanceRange_.x() != weightDistanceRange_.y())
  {

    Eigen::Matrix2d A;
    A << weightDistanceRange_.x() , 1 , weightDistanceRange_.y() ,1;
    const Eigen::Vector2d coeff = A.inverse() * weightRange_; 
    const double w = minDist_ * coeff.x() + coeff.y();
    weight_ = std::min(std::max(w,weightRange_.x()),weightRange_.y());
  }

  for(size_t i = 0 ; i < biTasks_.size() ; i++)
  {
    const double w = softMax(dist,-softMaxGain_,i);
    biTasks_[i]->weight(std::max( 0.,weight_ * w - deltaDistGain_ * exp(10 * distVel.back())));
  }

  // biTask_->updateHuman(hp_1,hp_2);



  ctl_.datastore().assign<std::vector<std::shared_ptr<mc_tasks::biRobotTeleopTask>>>( name() +"_tasks",biTasks_);

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

void BiRobotTeleoperation_Task::teardownLogger(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

  logger.removeLogEntry(name() + "_reference_weight");
  logger.removeLogEntry(name() + "_reference_distance");
  logger.removeLogEntry(name() + "_soft_max_gain");

}

void BiRobotTeleoperation_Task::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  for (auto & task : biTasks_)
  {
    ctl_.solver().removeTask(task);
  }
  ctl_.datastore().remove(name()+"_tasks");
  auto posture_1 = ctl_.getPostureTask(r1_name_);
  auto posture_2 = ctl_.getPostureTask(r2_name_);
  posture_1->weight(10);
  posture_2->weight(10);
  posture_1->stiffness(5);
  posture_2->stiffness(5);

  teardownLogger(ctl);

}

EXPORT_SINGLE_STATE("BiRobotTeleoperation_Task", BiRobotTeleoperation_Task)
