#include "BiRobotTeleoperation.h"
#include <mc_rbdyn/RobotLoader.h>

#include "yaml_path.h"

BiRobotTeleoperation::BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // std::cout << config_("states").dump() << std::endl;
  // std::cout << "//" << std::endl;
  // config_.load(mc_rtc::Configuration(BiRobotTask_CONFIG_PATH));

  // std::cout << config_("states").dump() << std::endl;

  external_robots_ = mc_rbdyn::Robots::make();

  datastore().make<mc_rbdyn::RobotsPtr>("external_robots",external_robots_);
  
  mc_rbdyn::Robot & human_1 = robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = robots().robot("human_2");
  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;
  updateHumanPose(human_1,hp_1);
  updateHumanPose(human_2,hp_2);
  datastore().make<bilateralTeleop::HumanPose>("human_1",hp_1);
  datastore().make<bilateralTeleop::HumanPose>("human_2",hp_2);

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

bool BiRobotTeleoperation::run()
{

  mc_rbdyn::Robot & human_1 = robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = robots().robot("human_2");

  bilateralTeleop::HumanPose hp_1;
  bilateralTeleop::HumanPose hp_2;
  updateHumanPose(human_1,hp_1);
  updateHumanPose(human_2,hp_2);
  datastore().assign<bilateralTeleop::HumanPose>("human_1",hp_1);
  datastore().assign<bilateralTeleop::HumanPose>("human_2",hp_2);

  return mc_control::fsm::Controller::run();
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_rbdyn::Robot & robot_2 = robots().robot("robot_2");

  mc_rbdyn::Robot & human_1 = robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = robots().robot("human_2");


  robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0)) * alignFeet(robot(),"Foot",robot_2,"Foot") );

  
  human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.6, 0., 0)) * alignFeet(robot_2,"Foot",human_1,"Sole") );
  human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.6, 0., 0)) * alignFeet(robot(),"Foot",human_2,"Sole") );

  mc_control::fsm::Controller::reset(reset_data);
}

sva::PTransformd alignFeet(const mc_rbdyn::Robot & robot_1,const std::string & surfaceSuffix_1, const mc_rbdyn::Robot & robot_2,const std::string & surfaceSuffix_2)
{
  const sva::PTransformd X_0_Fc1 = sva::PTransformd(robot_1.posW().rotation(),0.5 * ( robot_1.frame("Left" + surfaceSuffix_1).position().translation() 
                                                                                  + robot_1.frame("Right" + surfaceSuffix_1).position().translation()));


  const sva::PTransformd X_0_Fc2 = sva::PTransformd(robot_2.posW().rotation(),0.5 * ( robot_2.frame("Left" + surfaceSuffix_2).position().translation() 
                                                                                  + robot_2.frame("Right" + surfaceSuffix_2).position().translation()));

  const sva::PTransformd X_b2_fc2 = X_0_Fc2 * robot_2.posW().inv();

  return X_b2_fc2.inv() * X_0_Fc1;

}

