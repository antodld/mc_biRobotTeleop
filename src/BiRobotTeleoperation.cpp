#include "BiRobotTeleoperation.h"
#include "convexViz.h"
#include "yaml_path.h"
#include <mc_tasks/biRobotTeleopTask.h>
#include <mc_rtc/gui/Robot.h>

BiRobotTeleoperation::BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // std::cout << config_("states").dump() << std::endl;
  // std::cout << "//" << std::endl;
  // config_.load(mc_rtc::Configuration(BiRobotTask_CONFIG_PATH));

  external_robots_ = mc_rbdyn::Robots::make();

  hp_1_ = biRobotTeleop::HumanPose("human_1");
  hp_2_ = biRobotTeleop::HumanPose("human_2");

  hp_1_filtered_ = biRobotTeleop::HumanPose("human_1_filtered");
  hp_2_filtered_ = biRobotTeleop::HumanPose("human_2_filtered");

  if (config.has("human_1"))
  {
    hp_1_.setCvx(config("human_1")("convex"));
    hp_1_filtered_.setCvx(config("human_1")("convex"));
  }
  if (config.has("human_2"))
  {
    hp_2_.setCvx(config("human_2")("convex"));
    hp_2_filtered_.setCvx(config("human_2")("convex"));
  }

 

  global_config_.load(config);
  if(global_config_.has("Plugins"))
  {   
      std::vector<std::string> plugins;
      global_config_("Plugins",plugins);
      if( std::find(plugins.begin(), plugins.end(), "ROS") != plugins.end() )
      {
          use_ros_ = true;
          auto n = mc_rtc::ROSBridge::get_node_handle();
          sch_pub_ = n->advertise<visualization_msgs::MarkerArray>("sch_marker", 1000);
      }
  }
  
  config("distant_controller")("ip_",ip_);
  config("distant_controller")("pub_port",pub_port_);
  config("distant_controller")("sub_port",sub_port_);
  config("distant_controller")("human_name",distant_human_name_);
  config("local_controller")("human_name",local_human_name_);

  if(distant_human_name_ == "human_2")
  {
    hp_1_.addDataToGUI(*gui().get());
    distant_human_indx_ = 1;
  }  
  else
  {
    hp_2_.addDataToGUI(*gui().get());
  }


  distant_robot_name_ = (robot().name() == "robot_1") ? "robot_2" : "robot_1";

  hp_rec_.init(distant_human_name_, distant_robot_name_ ,"tcp://" + ip_ + ":" + std::to_string(pub_port_),
                        "tcp://" + ip_ + ":" + std::to_string(sub_port_));

  hp_rec_.setSimulatedDelay(1);

  gui()->addElement({"BiRobotTeleop"},mc_rtc::gui::Checkbox("Online",[this]() -> bool {return true;},[this](){}));
  gui()->addElement({"BiRobotTeleop"},mc_rtc::gui::Checkbox("Distant Controller Online",[this]() -> bool {return hp_rec_.online() ;},[this](){}));

  for (auto & f : robots().robot("robot_2").frames())
  {
    mc_rtc::log::info("panda frame {}",f);
  }  
  for (auto & c : robots().robot("robot_2").convexes())
  {
    mc_rtc::log::info("convex name {}",c.first);
  }
  for (auto & c : robots().robot("robot_1").convexes())
  {
    mc_rtc::log::info("robot_1 convex name {}",c.first);
  }

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

bool BiRobotTeleoperation::run()
{
  updateDistantHumanRobot();

  if(robots().hasRobot("human_1"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");
    updateHumanPose(human_1,hp_1_);
    updateHumanPose(human_2,hp_2_);
  }

  if(use_ros_)
  {
    size_t id = 0;
    markers_.markers.clear();
    for (int partInt = biRobotTeleop::Limbs::LeftHand ; partInt <= biRobotTeleop::Limbs::RightArm ; partInt++)
    {
        biRobotTeleop::Limbs part = static_cast<biRobotTeleop::Limbs>(partInt);
        auto cvx_1 = hp_1_.getConvex(part);
        auto cvx_2 = hp_2_.getConvex(part);
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_1_" + biRobotTeleop::limb2Str(part),id,cvx_1,sva::PTransformd::Identity()));
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_2_" + biRobotTeleop::limb2Str(part),id+1,cvx_2,sva::PTransformd::Identity()));
        id +=2;
    }
    
    sch_pub_.publish(markers_);
    
  }

  return mc_control::fsm::Controller::run();
}

void BiRobotTeleoperation::updateDistantHumanRobot()
{
  auto & h = getHumanPose(distant_human_indx_);
  h.updateHumanState(hp_rec_.getHumanPose());
  auto & robot = robots().robot(distant_robot_name_);
  if(hp_rec_.online() && robot.name() == hp_rec_.getRobot().name())
  {
    robot.mbc().q = hp_rec_.getRobot().mbc().q;
    robot.mbc().alpha = hp_rec_.getRobot().mbc().alpha;
    robot.posW(hp_rec_.getRobot().posW());
  } 
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  mc_rbdyn::Robot & robot_2 = robots().robot("robot_2");

  
  if(robot_2.module().name != "panda_default")
  {
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0)) * alignFeet(robot(),"Foot",robot_2,"Foot") );
  }
  else
  {
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0.)) * robot().posW() );
  }


  if(robots().hasRobot("human_1"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");
    
    human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., 0)) * alignFeet(robot(),"Foot",human_2,"Sole") );

    if(robot_2.module().name != "panda_default")
    {
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., -0.3)) * alignFeet(robot_2,"Foot",human_1,"Sole") );
    }
    else
    {
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4 + 0.6 + 0.4, 0., 0.)) * human_2.posW() );
    }
  }
  
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

