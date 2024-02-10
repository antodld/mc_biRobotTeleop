#include "BiRobotTeleoperation.h"
#include "convexViz.h"
#include "yaml_path.h"
#include <mc_tasks/biRobotTeleopTask.h>
#include <mc_rtc/gui/RobotMsg.h>
#include <RBDyn/FK.h>
#include <RBDyn/FD.h>
#include <RBDyn/FV.h>
#include <RBDyn/FA.h>
#include <RBDyn/ID.h>
#include <RBDyn/Coriolis.h>

BiRobotTeleoperation::BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // std::cout << config_("states").dump() << std::endl;
  // std::cout << "//" << std::endl;
  // config_.load(mc_rtc::Configuration(BiRobotTask_CONFIG_PATH));

  
  external_robots_ = mc_rbdyn::Robots::make();

  // const auto robot_2_indx = robots().robot("robot_2").robotIndex();
  // mc_solver::CollisionsConstraint robot_2_collision_cstr(robots(),robot_2_indx,robot_2_indx,dt); 
  // robot_2_collision_cstr.addCollisions(solver(), robots().robot(robot_2_indx).module().commonSelfCollisions());

  // solver().addConstraintSet(robot_2_collision_cstr);

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

  if(config.has("robot_limb_map"))
  {
    r_1_.load(config("robot_limb_map")("robot_1"));
    r_2_.load(config("robot_limb_map")("robot_2"));
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

  const std::string kinematics_intertial_datastoreFunc_name = "KinematicAnchorFrame::" + robot("robot_1").name();
  if(!datastore().has(kinematics_intertial_datastoreFunc_name))
  {
    datastore().make_call(
      kinematics_intertial_datastoreFunc_name, [this](const mc_rbdyn::Robot & robot)
      {return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5); });
  }

  int server_pub_port = config("server")("sub_port");
  int server_sub_port = config("server")("pub_port");
  std::string server_ip = config("server")("ip");
  server_.reset(new mc_control::ControllerServer(dt, dt, {"ipc:///tmp/hp_server_pub.ipc","tcp://"+ server_ip + ":" + std::to_string(server_pub_port)},
                                                         {"ipc:///tmp/hp_server_rep.ipc","tcp://"+ server_ip + ":" + std::to_string(server_sub_port)})
                                                         );
  
  config("distant_controller")("ip",ip_);
  config("distant_controller")("pub_port",pub_port_);
  config("distant_controller")("sub_port",sub_port_);
  config("distant_controller")("human_name",distant_human_name_);
  config("local_controller")("human_name",local_human_name_);

  if(distant_human_name_ == "human_2")
  {
    hp_1_.addDataToGUI(gui_builder_);
    distant_human_indx_ = 1;
  }  
  else
  {
    hp_2_.addDataToGUI(gui_builder_);
  }

  if(config("local_controller")("display_human_pose"))
  {
    hp_1_.addPoseToGUI(*gui().get());
    hp_2_.addPoseToGUI(*gui().get());
    
    hp_1_.addOffsetToGUI(*gui().get());
    hp_2_.addOffsetToGUI(*gui().get());
  }

  distant_robot_name_ = (robot().name() == "robot_1") ? "robot_2" : "robot_1";

  hp_rec_.init(distant_human_name_, distant_robot_name_ ,"tcp://" + ip_ + ":" + std::to_string(pub_port_),
                        "tcp://" + ip_ + ":" + std::to_string(sub_port_));

  hp_rec_.setSimulatedDelay(0);

  gui_builder_.addElement({"BiRobotTeleop"},mc_rtc::gui::Checkbox("Online",[this]() -> bool {return true;},[this](){}));
  gui_builder_.addElement({"BiRobotTeleop"},mc_rtc::gui::RobotMsg("robot",[this]() -> const mc_rbdyn::Robot & {return robot(); }));
  if(robots().robot("robot_2").module().name == "panda_default")
  {
    gui_builder_.addElement({"BiRobotTeleop"},mc_rtc::gui::RobotMsg("panda_robot",[this]() -> const mc_rbdyn::Robot & {return robots().robot("robot_2"); }));
  }
  gui()->addElement({"BiRobotTeleop"},mc_rtc::gui::Checkbox("Distant Controller Online",[this]() -> bool {return hp_rec_.online() ;},[this](){}));
  gui()->addElement({},mc_rtc::gui::Button("Emergency Stop : B",[this](){hardEmergency();}));

  for (auto & robot : realRobots())
  { 
    external_wrench_calib_.push_back(sva::ForceVecd::Zero());
    logger().addLogEntry(robot.name() + "_ext_force_base",[this,&robot]() -> const sva::ForceVecd {return getCalibratedExtWrench(robot);});
  }

  if(global_config_.has("Franka"))
  {
    gui()->addElement({"Robots"},
          mc_rtc::gui::Button("Reset robot_2",[this](){resetToRealRobot("robot_2");}));
    gui()->addElement({"Robots","External Wrench"},
          mc_rtc::gui::Button("Calibrate robot_1",[this](){CalibrateExtWrench(realRobot("robot_1"));}),
          mc_rtc::gui::ArrayLabel("robot_1 ext wrench",{"cx","cy","cz","fx","fy","fz"},
                                  [this]() -> sva::ForceVecd {return getCalibratedExtWrench(realRobot("robot_1"));}),
          mc_rtc::gui::Button("Calibrate robot_2",[this](){CalibrateExtWrench(realRobot("robot_2"));}),
          mc_rtc::gui::ArrayLabel("robot_2 ext wrench",{"cx","cy","cz","fx","fy","fz"},
                                  [this]() -> sva::ForceVecd {return getCalibratedExtWrench(realRobot("robot_2"));})

    );
  }

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

  if(!datastore().has("Replay::Log"))
  {
    addReplayLog(0);
    addReplayLog(1);
  }

  if(global_config_.has("Franka"))
  {
    if(global_config_("Franka")("ControlMode") == "Torque")
    {
      gui()->addElement({"Robots"},mc_rtc::gui::NumberInput("Close Loop gain",[this]() -> const double {return cl_gain_;},[this](const double k){cl_gain_ = k;}));
    }
  }

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

void BiRobotTeleoperation::resetToRealRobot(const std::string & name)
{
  mc_rtc::log::info("Reset {} state to real one",name);
  auto & robot = robots().robot(name);
  auto & realRobot = realRobots().robot(name);
  robot.mbc().q = realRobot.mbc().q;
  robot.mbc().alpha = realRobot.mbc().alpha;
  rbd::forwardKinematics(robot.mb(),robot.mbc());
  rbd::forwardVelocity(robot.mb(),robot.mbc());
  rbd::forwardAcceleration(robot.mb(),robot.mbc());
}

bool BiRobotTeleoperation::run()
{

  if(joystickButtonPressed(joystickButtonInputs::B))
  {
    hardEmergency();
  }
  if(joystickButtonPressed(joystickButtonInputs::X))
  {
    resetToRealRobot("robot_2");
  }

  if(hp_rec_.online())
  {  
    updateDistantHumanRobot();
  }

  if(robots().hasRobot("human_1"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");
    updateHumanPose(human_1,hp_1_);
    updateHumanPose(human_2,hp_2_);
  }

  //Panda Close Loop
  if(global_config_.has("Franka"))
  {
    if(global_config_("Franka")("ControlMode") == "Torque")
    {
      auto & robot = robots().robot("robot_2");
      auto & realRobot = realRobots().robot("robot_2");
      rbd::ForwardDynamics fd(robot.mb());
      fd.computeH(robot.mb(),realRobot.mbc());
      const Eigen::VectorXd alpha_r = rbd::paramToVector(robot.mb(),robot.mbc().alpha);
      const Eigen::VectorXd alpha = rbd::paramToVector(robot.mb(),realRobot.mbc().alpha);
      const auto s = alpha_r - alpha;
      rbd::Coriolis C(realRobot.mb());
      const auto C_mat = C.coriolis(realRobot.mb(),realRobot.mbc());
      const Eigen::MatrixXd K = cl_gain_ * fd.H();
      robot.mbc().q = realRobot.mbc().q;
      robot.mbc().alpha = realRobot.mbc().alpha;
      rbd::forwardKinematics(robot.mb(),robot.mbc());
      rbd::forwardVelocity(robot.mb(),robot.mbc());
      rbd::forwardAcceleration(robot.mb(),robot.mbc());
      rbd::InverseDynamics id(robot.mb());
      id.inverseDynamics(robot.mb(),robot.mbc());

      robot.mbc().jointTorque = rbd::vectorToParam(robot.mb(), rbd::paramToVector(robot.mb(),robot.mbc().jointTorque) +  (C_mat + K) * s );

      // mc_rtc::log::info(rbd::paramToVector(robot.mb(),robot.mbc().jointTorque));
    }
  }

  if(use_ros_)
  {
    size_t id = 0;
    markers_.markers.clear();
    for (int partInt = biRobotTeleop::Limbs::LeftHand ; partInt <= biRobotTeleop::Limbs::RightArm ; partInt++)
    {
        biRobotTeleop::Limbs part = static_cast<biRobotTeleop::Limbs>(partInt);
        // const auto cvx_1 = hp_1_.getConvex(part);
        // const auto cvx_2 = hp_2_.getConvex(part);
        const auto cvx_1 = hp_1_filtered_.getConvex(part);
        const auto cvx_2 = hp_2_filtered_.getConvex(part);
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_1_" + biRobotTeleop::limb2Str(part),id,cvx_1,sva::PTransformd::Identity()));
        markers_.markers.push_back(fromCylinder("control/env_1/ground","human_2_" + biRobotTeleop::limb2Str(part),id+1,cvx_2,sva::PTransformd::Identity()));
        id +=2;
    }
    
    sch_pub_.publish(markers_);
    
  }

  server_->publish(gui_builder_);
  hp_rec_.runAndUpdate();

  ctl_count_++;

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

void BiRobotTeleoperation::addReplayLog(const int indx)
{
  const auto & h = getHumanPose(indx);
  for (int limb_indx = 0 ; limb_indx <= biRobotTeleop::Limbs::RightArm ; limb_indx++)
  {

    const auto limb = static_cast<biRobotTeleop::Limbs>(limb_indx);
    logger().addLogEntry("Replay_"+h.name() +"_"+ biRobotTeleop::limb2Str(limb) + "_pose",[this,indx,limb]() -> const sva::PTransformd &{return getHumanPose(indx).getPose(limb);});
    logger().addLogEntry("Replay_"+h.name() +"_"+ biRobotTeleop::limb2Str(limb) + "_vel",[this,indx,limb]() -> const sva::MotionVecd &{return getHumanPose(indx).getVel(limb);});
    logger().addLogEntry("Replay_"+h.name() +"_"+ biRobotTeleop::limb2Str(limb) + "_acc",[this,indx,limb]() -> const sva::MotionVecd &{return getHumanPose(indx).getAcc(limb);});
    logger().addLogEntry("Replay_"+h.name() +"_"+ biRobotTeleop::limb2Str(limb) + "_active",[this,indx,limb]() -> const bool {return getHumanPose(indx).limbActive(limb);});


  }

}

bool BiRobotTeleoperation::joystickButtonPressed(const joystickButtonInputs input)
{
  bool joystick_online = false;

  if(datastore().has("Joystick::connected"))
  {
    joystick_online = datastore().get<bool>("Joystick::connected");
  }

  if(joystick_online)
  {
    auto & buttonEvent_func = datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::ButtonEvent");
    auto & button_func = datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::Button");
    return  button_func(input) && buttonEvent_func(input);
  }
  return false;
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  auto & robot_2 = robots().robot("robot_2");
  auto & robot_1 = robots().robot("robot_1");

  robot_2.posW(sva::PTransformd::Identity());

  
  if(robot_2.module().name != "panda_default")
  {
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0)) * alignFeet(robot_1,"Foot",robot_2,"Foot") );
  }
  else
  {
    // robot_2.mbc().gravity.setZero();
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0.)) * robot_2.posW() );
    realRobots().robot("robot_2").posW(robot_2.posW());
    // robot_2.posW(sva::PTransformd(sva::RotZ(M_PI_2), Eigen::Vector3d(-0.6, 0., 0.)) * robot().posW() );
    
  }



  if(robots().hasRobot("human_1"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");

    human_1.posW(sva::PTransformd::Identity());
    human_2.posW(sva::PTransformd::Identity());
    
    human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., 0)) * alignFeet(robot_1,"Foot",human_2,"Sole") );

    if(robot_2.module().name != "panda_default")
    {
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., -0.3)) * alignFeet(robot_2,"Foot",human_1,"Sole") );
    }
    else
    {
      // human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4 + 0.6 + 0.4, 0., 0.)) * human_2.posW() );
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0, 0.15)) * robot_2.posW() ); //0.7
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

