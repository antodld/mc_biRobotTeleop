#include "BiRobotTeleoperation.h"
#include "convexViz.h"
#include "yaml_path.h"
#include <mc_tasks/biRobotTeleopTask.h>

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

  updateHumanPose(human_1,hp_1_);
  updateHumanPose(human_2,hp_2_);
  datastore().make<biRobotTeleop::HumanPose>("human_1",hp_1_);
  datastore().make<biRobotTeleop::HumanPose>("human_2",hp_2_);

  auto n = mc_rtc::ROSBridge::get_node_handle();
  sch_pub_ = n->advertise<visualization_msgs::MarkerArray>("sch_marker", 1000);

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

bool BiRobotTeleoperation::run()
{

  mc_rbdyn::Robot & human_1 = robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = robots().robot("human_2");


  updateHumanPose(human_1,hp_1_);
  updateHumanPose(human_2,hp_2_);
  datastore().assign<biRobotTeleop::HumanPose>("human_1",hp_1_);
  datastore().assign<biRobotTeleop::HumanPose>("human_2",hp_2_);

  if (datastore().has("BiTeleopTask_1_tasks"))
  {
    markers_.markers.clear();
    std::vector<std::shared_ptr<mc_tasks::biRobotTeleopTask>> tasks;
    datastore().get<std::vector<std::shared_ptr<mc_tasks::biRobotTeleopTask>>>("BiTeleopTask_1_tasks",tasks);

    auto hp = tasks[0]->getHumanPose();
    hp_1_.setOffset(hp[0].getOffset()); hp_2_.setOffset(hp[1].getOffset());
    size_t id = 0;
    for (int partInt = biRobotTeleop::Limbs::LeftHand ; partInt <= biRobotTeleop::Limbs::RightArm ; partInt++)
    {
        biRobotTeleop::Limbs part = static_cast<biRobotTeleop::Limbs>(partInt);
        auto cvx_1 = hp[0].getConvex(part);
        auto cvx_2 = hp[1].getConvex(part);
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_1_" + biRobotTeleop::limb2Str(part),id,cvx_1,sva::PTransformd::Identity()));
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_2_" + biRobotTeleop::limb2Str(part),id+1,cvx_2,sva::PTransformd::Identity()));
        id +=2;
    }
    
    sch_pub_.publish(markers_);
  }


  return mc_control::fsm::Controller::run();
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_rbdyn::Robot & robot_2 = robots().robot("robot_2");

  mc_rbdyn::Robot & human_1 = robots().robot("human_1");
  mc_rbdyn::Robot & human_2 = robots().robot("human_2");


  robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0)) * alignFeet(robot(),"Foot",robot_2,"Foot") );

  
  human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., -0.3)) * alignFeet(robot_2,"Foot",human_1,"Sole") );
  human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., 0)) * alignFeet(robot(),"Foot",human_2,"Sole") );

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

