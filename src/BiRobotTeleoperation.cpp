#include "BiRobotTeleoperation.h"

#include <mc_control/ControllerServer.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/gui/RobotMsg.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/biRobotTeleopTask.h>

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

#include "config.h"
#include "convexGUI.h"
#include "yaml_path.h"

/**
 * This function modifies the FSM configuration to add features specific to the BiRobotTeleoperation controller:
 * - IncludeStates: allows to include additional states from external configuration files
 * - IncludeObservers: allows to include additional observers from external configuration files
 * - IncludGlobals: allows to include additional global configuration from external configuration files
 *
 * We might want to adapt what is included based on the selected mode.
 */
static inline mc_rtc::Configuration patchConfig(const mc_rtc::Configuration & config)
{
  using Mode = BiRobotTeleoperation::Mode;
  mc_rtc::Configuration patchedConfig = config;
  auto mode = BiRobotTeleoperation::from_string(config("mode", std::string{"None"}));
  if(mode == Mode::None)
  {
    mc_rtc::log::error_and_throw(
        "BiRobotTeleoperation mode is None, please specify a valid mode in the FSM configuration");
  }
  auto modeStr = BiRobotTeleoperation::to_string(mode);

  auto includeNames = config("IncludeStates", std::vector<std::string>{});
  std::for_each(includeNames.begin(), includeNames.end(),
                [&patchedConfig](const std::string & includeName)
                {
                  mc_rtc::ConfigurationFile includedState(std::string{biRobotTeleop::ETC_PATH_BUILD} + includeName);
                  mc_rtc::log::info("Loaded states configuration from {}", includedState.path());
                  patchedConfig("states").load(includedState);
                });

  auto includeObservers = config("IncludeObservers", std::vector<std::string>{});
  std::for_each(
      includeObservers.begin(), includeObservers.end(),
      [&patchedConfig](const std::string & includeName)
      {
        mc_rtc::ConfigurationFile includedObserver(std::string{biRobotTeleop::ETC_PATH_BUILD} + includeName);
        mc_rtc::log::info("Loaded observer configuration from {}", includedObserver.path());
        patchedConfig("ObserverPipelines").load(includedObserver("ObserverPipelines", mc_rtc::Configuration{}));
        patchedConfig("observers").load(includedObserver);
      });

  auto includeGlobals = config("IncludeGlobals", std::vector<std::string>{});
  std::for_each(includeGlobals.begin(), includeGlobals.end(),
                [&patchedConfig](const std::string & includeName)
                {
                  mc_rtc::ConfigurationFile includedGlobal(std::string{biRobotTeleop::ETC_PATH_BUILD} + includeName);
                  mc_rtc::log::info("Loaded global configuration from {}", includedGlobal.path());
                  patchedConfig.load(includedGlobal);
                });

  mc_rtc::ConfigurationFile includedPlugins(std::string{biRobotTeleop::ETC_PATH} + "Plugins.yaml");
  if(auto modePlugin = includedPlugins.find(modeStr))
  {
    patchedConfig.load(*modePlugin);
  }

  return patchedConfig;
}

BiRobotTeleoperation::BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, patchConfig(config))
{
}

void BiRobotTeleoperation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  mode_ = from_string(config()("mode"));
  mc_rtc::log::info("[{}] Mode is {}", name_, to_string(mode_));

  if(mode_ == Mode::SimulationSingle)
  {
    config().add("mode", "SimulationSingle");

    auto rm = mc_rbdyn::RobotLoader::get_robot_module("human");
    mc_rtc::log::info("Loading robot 'human_1' from module '{}'", rm->name);
    loadRobot(rm, "human_1");
    mc_rtc::log::info("Loading robot 'human_2' from module '{}'", rm->name);
    loadRobot(rm, "human_2");

    config()("human_sim").add("active", true);

    // Load mc_HumanMap.yaml
    mc_rtc::log::info("Loading human map configuration from mc_humanMap_SimulationSingle.yaml");
    mc_rtc::ConfigurationFile humanMapConfig(std::string{biRobotTeleop::ETC_PATH_BUILD}
                                             + "mc_humanMap_SimulationSingle.yaml");
    config().load(humanMapConfig);
  }
  else if(mode_ == Mode::SingleVR || mode_ == Mode::DualVR)
  {
    mc_rtc::log::info("[{}] Selected Mode SingleVR", name_);
    config()("human_sim").add("active", false);

    auto estimationModule = config()("estimation_module", std::string{"simple_human"});
    mc_rtc::Configuration humanMapConfig;
    if(estimationModule == "simple_human")
    { // Load HumanMap.yaml
      mc_rtc::log::info("Loading human map configuration for module {} from HumanMap.yaml", estimationModule);
      config().load(mc_rtc::Configuration(std::string{biRobotTeleop::ETC_PATH_BUILD} + "HumanMap.yaml"));
    }
    else
    { // for mc_human
      mc_rtc::log::info("Loading human map configuration for module {} from mc_humanMap.yaml", estimationModule);
      config().load(std::string{biRobotTeleop::ETC_PATH_BUILD} + "mc_humanMap.yaml");
    }
  }

  init_();
  reset_();

  // solver().addConstraintSet(std::make_unique<mc_solver::KinematicsConstraint>(robots(),
  // robots().robotIndex("human_1"), solver().dt()));
  // solver().addConstraintSet(std::make_unique<mc_solver::KinematicsConstraint>(robots(),
  // robots().robotIndex("human_2"), solver().dt()));

  // const auto h_2_indx = robots().robot("human_2").robotIndex();
  // mc_solver::CollisionsConstraint h_2_collision_cstr(robots(),h_2_indx,h_2_indx,solver().dt());
  // h_2_collision_cstr.addCollisions(solver(), robots().robot(h_2_indx).module().commonSelfCollisions());
  // solver().addConstraintSet(h_2_collision_cstr);

  // mc_rtc::log::info("added kinematics constraints for humans");

  run();
}

void BiRobotTeleoperation::init_()
{
  auto config = this->config();
  const auto dt = this->timeStep;

  // std::cout << config_("states").dump() << std::endl;
  // std::cout << "//" << std::endl;
  // config_.load(mc_rtc::Configuration(BiRobotTask_CONFIG_PATH));

  external_robots_ = mc_rbdyn::Robots::make();
  // FIXME! the storage insde Robots depends on std::vector storage for robot modules, mbgs, mbcs etc
  // However we return references to this underlying storage.
  // Thus dynamically adding a new robot can cause reallocation, and thus invalidate those references
  //
  // In our case here, adding robots in HumanPoseEstmation will cause a crash in the RobotModuleUpdate plugin when
  // adding Visual shapes from the robot module to the gui (since robot.module() returns a reference to robot modules
  // stored in an std::vector within Robots.
  external_robots_->reserve(10);

  // const auto robot_2_indx = robots().robot("robot_2").robotIndex();
  // mc_solver::CollisionsConstraint robot_2_collision_cstr(robots(),robot_2_indx,robot_2_indx,dt);
  // robot_2_collision_cstr.addCollisions(solver(), robots().robot(robot_2_indx).module().commonSelfCollisions());

  // solver().addConstraintSet(robot_2_collision_cstr);

  hp_1_ = biRobotTeleop::HumanPose("human_1");
  hp_2_ = biRobotTeleop::HumanPose("human_2");

  hp_1_filtered_ = biRobotTeleop::HumanPose("human_1_filtered");
  hp_2_filtered_ = biRobotTeleop::HumanPose("human_2_filtered");

  if(auto collision_with_joint_selection = config("collisions_with_joint_selection", mc_rtc::Configuration{})
                                               .find(robots().robot(robot().name()).module().name))
  {
    create_collision_cstr(*collision_with_joint_selection);
  }

  auto setConvex =
      [&config](const std::string & robot, biRobotTeleop::HumanPose & hp, biRobotTeleop::HumanPose & hp_filtered)
  {
    hp.setCvx(config(robot)("convex"));
    hp_filtered.setCvx(config(robot)("convex"));
    hp.setLimbMap(config("human")("limb_map"));
    hp_filtered.setLimbMap(config("human")("limb_map"));
  };
  setConvex("human_1", hp_1_, hp_1_filtered_);
  setConvex("human_2", hp_2_, hp_2_filtered_);

  if(config.find("robot_limb_map"))
  {
    r_1_.load(*(config("robot_limb_map", mc_rtc::Configuration{}).find(robots().robot("robot_1").module().name)));
    r_2_.load(*(config("robot_limb_map", mc_rtc::Configuration{}).find(robots().robot("robot_2").module().name)));
  }

  global_config_.load(config);

  const std::string kinematics_intertial_datastoreFunc_name = "KinematicAnchorFrame::" + robot("robot_1").name();
  if(!datastore().has(kinematics_intertial_datastoreFunc_name))
  {
    datastore().make_call(
        kinematics_intertial_datastoreFunc_name, [this](const mc_rbdyn::Robot & robot)
        { return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5); });
  }

  int server_pub_port = config("server")("pub_port");
  int server_sub_port = config("server")("sub_port");

  config("distant_controller")("ip", ip_);
  config("distant_controller")("pub_port", pub_port_);
  config("distant_controller")("sub_port", sub_port_);
  config("distant_controller")("human_name", distant_human_name_);
  config("local_controller")("human_name", local_human_name_);

  std::string server_ip = config("server")("ip");
  server_.reset(new mc_control::ControllerServer(dt, dt, {"tcp://" + server_ip + ":" + std::to_string(server_pub_port)},
                                                 {"tcp://" + server_ip + ":" + std::to_string(server_sub_port)}));

  if(distant_human_name_ == "human_2")
  {
    hp_1_.addDataToGUI(gui_builder_);
    distant_human_indx_ = 1;
  }
  else
  {
    hp_2_.addDataToGUI(gui_builder_);
  }

  if(config("local_controller")("display_human_pose", true))
  {
    hp_1_.addPoseToGUI(*gui().get(), false);
    hp_2_.addPoseToGUI(*gui().get(), false);

    hp_1_.addOffsetToGUI(*gui().get());
    hp_2_.addOffsetToGUI(*gui().get());
  }

  distant_robot_name_ = (robot().name() == "robot_1") ? "robot_2" : "robot_1";

  using namespace mc_rtc::gui;
  hp_rec_.init("receiver main", distant_human_name_, distant_robot_name_,
               "tcp://" + ip_ + ":" + std::to_string(pub_port_), "tcp://" + ip_ + ":" + std::to_string(sub_port_));
  hp_rec_.startConnection();

  hp_rec_.subsbscribe("Emergency", Elements::Checkbox, {"BiRobotTeleop"}, "Emergency");
  hp_rec_.subsbscribe("A", Elements::Checkbox, {"BiRobotTeleop"}, "A");
  hp_rec_.subsbscribe("Y", Elements::Checkbox, {"BiRobotTeleop"}, "Y");

  hp_rec_.setSimulatedDelay(0);

  gui_builder_.addElement(
      {"BiRobotTeleop"},
      Checkbox(
          "Online", [this]() -> bool { return true; }, [this]() {}),
      RobotMsg(robot().name(), [this]() -> const mc_rbdyn::Robot & { return robot(); }),
      Checkbox(
          "Emergency", [this]() -> const bool { return emergency_; }, [this]() {}),
      Checkbox(
          "A", [this]() -> const bool { return joystickButtonPressed(joystickButtonInputs::A); }, [this]() {}),
      Checkbox("Y", [this]() -> const bool { return joystickButtonPressed(joystickButtonInputs::Y); }, [this]() {}));

  if(robots().robot("robot_2").module().name == "panda_default")
  {
    gui_builder_.addElement({"BiRobotTeleop"}, RobotMsg("panda_robot", [this]() -> const mc_rbdyn::Robot &
                                                        { return robots().robot("robot_2"); }));
  }
  gui()->addElement(
      {"BiRobotTeleop"},
      mc_rtc::gui::Checkbox("Distant Controller Online", [this]() -> bool { return hp_rec_.online(); }, [this]() {}));
  gui()->addElement({}, mc_rtc::gui::Button("Emergency Stop : B", [this]() { hardEmergency(); }));

  for(auto & robot : realRobots())
  {
    external_wrench_calib_.push_back(sva::ForceVecd::Zero());
    logger().addLogEntry(robot.name() + "_ext_force_base",
                         [this, &robot]() -> const sva::ForceVecd { return getCalibratedExtWrench(robot); });
  }
  logger().addLogEntry("robot_1_ext_force_base_gt",
                       [this]() -> const sva::ForceVecd { return getExtWrenchGT(realRobot("robot_1"), "LeftHand"); });
  logger().addLogEntry("BiRobotTeleop_distantController_online", [this]() -> bool { return hp_rec_.online(); });

  if(global_config_.has("Franka"))
  {
    gui()->addElement({"Robots"}, mc_rtc::gui::Button("Reset robot_2", [this]() { resetToRealRobot("robot_2"); }),
                      mc_rtc::gui::Button("Reset robot_1", [this]() { resetToRealRobot("robot_1"); }));
    gui()->addElement(
        {"Robots", "External Wrench"},
        mc_rtc::gui::Button("Calibrate robot_1", [this]() { CalibrateExtWrench(realRobot("robot_1")); }),
        mc_rtc::gui::ArrayLabel("robot_1 ext wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                [this]() -> sva::ForceVecd { return getCalibratedExtWrench(realRobot("robot_1")); }),
        mc_rtc::gui::Button("Calibrate robot_2", [this]() { CalibrateExtWrench(realRobot("robot_2")); }),
        mc_rtc::gui::ArrayLabel("robot_2 ext wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                [this]() -> sva::ForceVecd { return getCalibratedExtWrench(realRobot("robot_2")); })

    );
  }

  // for (auto & f : robots().robot("robot_2").frames())
  // {
  //   mc_rtc::log::info("panda frame {}",f);
  // }
  // for (auto & c : robots().robot("robot_2").convexes())
  // {
  //   mc_rtc::log::info("convex name {}",c.first);
  // }
  // for (auto & c : robots().robot("robot_1").convexes())
  // {
  //   mc_rtc::log::info("robot_1 convex name {}",c.first);
  // }

  if(!datastore().has("Replay::Log"))
  {
    addReplayLog(0);
    addReplayLog(1);
  }

  if(global_config_.has("Franka")
     && (robot("robot_1").module().name == "panda" || robot("robot_2").module().name == "panda"))
  {
    if(global_config_("Franka")("ControlMode") == "Torque")
    {
      gui()->addElement({"Robots"}, mc_rtc::gui::NumberInput(
                                        "Close Loop gain", [this]() -> const double { return cl_gain_; },
                                        [this](const double k) { cl_gain_ = k; }));
    }
  }

  mc_rtc::log::success("BiRobotTeleoperation init done ");
}

void BiRobotTeleoperation::resetToRealRobot(const std::string & name)
{
  mc_rtc::log::info("Reset {} state to real one", name);
  auto & robot = robots().robot(name);
  auto & realRobot = realRobots().robot(name);
  robot.mbc().q = realRobot.mbc().q;
  robot.mbc().alpha = realRobot.mbc().alpha;
  rbd::forwardKinematics(robot.mb(), robot.mbc());
  rbd::forwardVelocity(robot.mb(), robot.mbc());
  rbd::forwardAcceleration(robot.mb(), robot.mbc());
}

void BiRobotTeleoperation::create_collision_cstr(const mc_rtc::Configuration & config)
{
  std::vector<mc_rbdyn::Collision> collisions;
  auto robot_bodies = config("simplified_all_bodies", std::vector<std::string>{});
  double iDist = config("iDist");
  double sDist = config("sDist");
  double default_iDist = iDist;
  double default_sDist = sDist;
  int cstr_set_indx = 0;

  if(robot_bodies.empty())
  {
    for(const auto & bd : robot().module().mb.bodies())
    {
      robot_bodies.push_back(bd.name());
    }
  }

  std::vector<std::string> robot_all_bodies = robot_bodies;

  auto collisionConf = mc_rtc::fromVectorOrElement(config, "cstr_sets", std::vector<mc_rtc::Configuration>{});

  for(auto & conf : collisionConf)
  {
    mc_rtc::log::info("adding set {}", cstr_set_indx);
    conf("iDist", iDist);
    conf("sDist", sDist);
    auto bodies_1 = conf("b1", std::vector<std::string>{});
    if(bodies_1.empty())
    {
      bodies_1 = robot_bodies;
    }
    auto bodies_2 = conf("b2", std::vector<std::string>{});
    if(bodies_2.empty())
    {
      bodies_2 = robot_all_bodies;
    }
    std::vector<std::string> joints = conf("joints");
    for(auto it = joints.begin(); it != joints.end();)
    {
      if(!this->robot().hasJoint(*it))
      {
        mc_rtc::log::error("Discarding joint {} because it does not exist in {}", *it, this->robot().name());
        it = joints.erase(it);
      }
      else
      {
        ++it;
      }
    }

    for(auto bd1 = std::begin(bodies_1); bd1 != std::end(bodies_1); bd1++)
    {
      for(auto bd2 = std::begin(bodies_2); bd2 != std::end(bodies_2); bd2++)
      {
        if(*bd1 != *bd2)
        {
          if(!robot().hasBody(*bd1))
          {
            mc_rtc::log::error("Discarding collision with {} because it does not exist in {}", *bd1, robot().name());
          }
          else if(!robot().hasBody(*bd2))
          {
            mc_rtc::log::error("Discarding collision with {} because it does not exist in {}", *bd2, robot().name());
          }
          else
          {
            collisions.push_back(mc_rbdyn::Collision(*bd1, *bd2, iDist, sDist, 0., joints, {}));
          }
        }
      }
      robot_bodies.erase(std::remove(robot_bodies.begin(), robot_bodies.end(), *bd1), robot_bodies.end());
    }

    iDist = default_iDist;
    sDist = default_sDist;
    cstr_set_indx += 1;
  }
  this->addCollisions(this->robot().name(), this->robot().name(), collisions);
  mc_rtc::log::success("[{}] Self Collisions added", name_);
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
  hp_rec_.update();

  if(hp_rec_.online())
  {
    updateDistantHumanRobot();
    bool distant_emergency = false;
    hp_rec_.getSubscribedData<bool>(distant_emergency, "Emergency");
    if(distant_emergency)
    {
      mc_rtc::log::critical("Distant Emergency triggered");
      emergency_ = distant_emergency;
    }
  }

  if(robots().hasRobot("human_1") && (robot("human_1").module().name == "human"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");
    updateHumanPose(human_1, hp_1_);
    updateHumanPose(human_2, hp_2_);
  }

  // Panda Close Loop
  if(global_config_.has("Franka")
     && (robot("robot_1").module().name == "panda" || robot("robot_2").module().name == "panda"))
  {
    if(global_config_("Franka")("ControlMode") == "Torque")
    {
      auto & robot = robots().robot("robot_2");
      auto & realRobot = realRobots().robot("robot_2");
      rbd::ForwardDynamics fd(robot.mb());
      fd.computeH(robot.mb(), realRobot.mbc());
      const Eigen::VectorXd alpha_r = rbd::paramToVector(robot.mb(), robot.mbc().alpha);
      const Eigen::VectorXd alpha = rbd::paramToVector(robot.mb(), realRobot.mbc().alpha);
      const auto s = alpha_r - alpha;
      rbd::Coriolis C(realRobot.mb());
      const auto C_mat = C.coriolis(realRobot.mb(), realRobot.mbc());
      const Eigen::MatrixXd K = cl_gain_ * fd.H();
      robot.mbc().q = realRobot.mbc().q;
      robot.mbc().alpha = realRobot.mbc().alpha;
      rbd::forwardKinematics(robot.mb(), robot.mbc());
      rbd::forwardVelocity(robot.mb(), robot.mbc());
      rbd::forwardAcceleration(robot.mb(), robot.mbc());
      rbd::InverseDynamics id(robot.mb());
      id.inverseDynamics(robot.mb(), robot.mbc());

      robot.mbc().jointTorque =
          rbd::vectorToParam(robot.mb(), rbd::paramToVector(robot.mb(), robot.mbc().jointTorque) + (C_mat + K) * s);

      // mc_rtc::log::info(rbd::paramToVector(robot.mb(),robot.mbc().jointTorque));
    }
  }

  // FIXME:
  // This code replaces the old ROS convex publishing
  // - Do not constantly remove/recreate GUI elements
  // - Do not allocate memory for convexes all the time -> modify HumanPose to store them
  // auto publishConvex = [&, this]()
  // {
  //   gui()->removeCategory({"BiRobotTeleop", "Convexes"});
  //   auto addConvex = [&, this](biRobotTeleop::HumanPose & hp, const std::string & robotName, biRobotTeleop::Limbs
  //   part)
  //   {
  //     const auto & human_1 = robots().robot("human_1");
  //     // XXX:can we modify HumanPose instead?
  //     const auto cvx_1 = std::shared_ptr<sch::S_Object>(hp.getConvex(part).clone());
  //     const auto & ground = robots().robot("ground");
  //     birobot_teleop::gui::addConvexToGUI(*gui(), {"BiRobotTeleop", "Convexes"}, ground, cvx_1,
  //                                         robotName + "_" + biRobotTeleop::limb2Str(part), /* convex name */
  //                                         "ground", /* body name */
  //                                         mc_rbdyn::gui::defaultConvexConfig /* config */
  //     );
  //   };
  //
  //   for(int partInt = biRobotTeleop::Limbs::LeftHand; partInt <= biRobotTeleop::Limbs::RightArm; partInt++)
  //   {
  //     biRobotTeleop::Limbs part = static_cast<biRobotTeleop::Limbs>(partInt);
  //     // const auto cvx_1 = hp_1_.getConvex(part);
  //     // const auto cvx_2 = hp_2_.getConvex(part);
  //     // markers_.markers.push_back(fromCylinder("control/env_1/ground","human_1_" +
  //     // biRobotTeleop::limb2Str(part),id,cvx_1,sva::PTransformd::Identity()));
  //     // markers_.markers.push_back(fromCylinder("control/env_1/ground","human_2_" +
  //     // biRobotTeleop::limb2Str(part),id+1,cvx_2,sva::PTransformd::Identity()));
  //     addConvex(hp_1_filtered_, "human_1", part);
  //     addConvex(hp_2_filtered_, "human_2", part);
  //   }
  // };
  // XXX: disable for now as not implemented in rviz client
  // publishConvex();

  server_->publish(gui_builder_);

  ctl_count_++;

  auto & robot = robots().robot(distant_robot_name_);
  // mc_rtc::log::info("q\n{}\nqd\n{}\nqdd\n{}",rbd::paramToVector(robot.mb(),robot.mbc().q),rbd::paramToVector(robot.mb(),robot.mbc().alpha),rbd::paramToVector(robot.mb(),robot.mbc().alphaD));
  // mc_rtc::log::info("distant : robot dof {}, alpha size {}, alphaD size
  // {}",robot.mb().nrDof(),robot.mbc().alpha.size(),robot.mbc().alphaD.size()); mc_rtc::log::info("local : robot dof
  // {}, alpha size {}, alphaD size
  // {}",robots().robot().mb().nrDof(),robots().robot().mbc().alpha.size(),robots().robot().mbc().alphaD.size());

  return mc_control::fsm::Controller::run() && !emergency_;
}

void BiRobotTeleoperation::updateDistantHumanRobot()
{
  auto & h = getHumanPose(distant_human_indx_);
  h.updateHumanState(hp_rec_.getHumanPose());
  auto & robot = robots().robot(distant_robot_name_);
  if(hp_rec_.online() && robot.name() == hp_rec_.robotName())
  {
    hp_rec_.updateRobot(robot);
  }
}

void BiRobotTeleoperation::addReplayLog(const int indx)
{
  const auto & h = getHumanPose(indx);
  for(int limb_indx = 0; limb_indx <= biRobotTeleop::Limbs::RightArm; limb_indx++)
  {

    const auto limb = static_cast<biRobotTeleop::Limbs>(limb_indx);
    logger().addLogEntry("Replay_" + h.name() + "_" + biRobotTeleop::limb2Str(limb) + "_pose",
                         [this, indx, limb]() -> const sva::PTransformd & { return getHumanPose(indx).getPose(limb); });
    logger().addLogEntry("Replay_" + h.name() + "_" + biRobotTeleop::limb2Str(limb) + "_vel",
                         [this, indx, limb]() -> const sva::MotionVecd & { return getHumanPose(indx).getVel(limb); });
    logger().addLogEntry("Replay_" + h.name() + "_" + biRobotTeleop::limb2Str(limb) + "_acc",
                         [this, indx, limb]() -> const sva::MotionVecd & { return getHumanPose(indx).getAcc(limb); });
    logger().addLogEntry("Replay_" + h.name() + "_" + biRobotTeleop::limb2Str(limb) + "_active",
                         [this, indx, limb]() -> const bool { return getHumanPose(indx).limbActive(limb); });
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
    return button_func(input) && buttonEvent_func(input);
  }
  return false;
}

void BiRobotTeleoperation::reset_()
{
  auto & robot_2 = robots().robot("robot_2");
  auto & robot_1 = robots().robot("robot_1");

  robot_2.posW(sva::PTransformd::Identity());

  if(robot_2.module().name != "panda_default")
  {
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0))
                 * alignFeet(robot_1, "Foot", robot_2, "Foot"));
  }
  else
  {
    // robot_2.mbc().gravity.setZero();
    robot_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(-0.6, 0., 0.)) * robot_2.posW());
    realRobots().robot("robot_2").posW(robot_2.posW());
    // robot_2.posW(sva::PTransformd(sva::RotZ(M_PI_2), Eigen::Vector3d(-0.6, 0., 0.)) * robot().posW() );
  }

  if(robots().hasRobot("human_1") && (robot("human_1").module().name == "human"))
  {
    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");

    human_1.posW(sva::PTransformd::Identity());
    human_2.posW(sva::PTransformd::Identity());

    human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.6, 0., 0))
                 * alignFeet(robot_1, "Foot", human_2, "Sole"));

    if(robot_2.module().name != "panda_default")
    {
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.6, 0., -0.2))
                   * alignFeet(robot_2, "Foot", human_1, "Sole"));
    }
    else
    {
      // human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4 + 0.6 + 0.4, 0., 0.)) * human_2.posW() );
      human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0, 0.15)) * robot_2.posW()); // 0.7
    }
  }

  else if(mode_ == Mode::SimulationSingle)
  {

    mc_rbdyn::Robot & human_1 = robots().robot("human_1");
    mc_rbdyn::Robot & human_2 = robots().robot("human_2");

    human_1.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0, 0.15)) * robot_2.posW()); // 0.7
    human_2.posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0., 0.15)) * robot_1.posW());

    solver().addConstraintSet(
        std::make_unique<mc_solver::KinematicsConstraint>(robots(), robots().robotIndex("human_2"), solver().dt()));
  }
}

sva::PTransformd alignFeet(const mc_rbdyn::Robot & robot_1,
                           const std::string & surfaceSuffix_1,
                           const mc_rbdyn::Robot & robot_2,
                           const std::string & surfaceSuffix_2)
{
  const sva::PTransformd X_0_Fc1 = sva::PTransformd(
      robot_1.posW().rotation(), 0.5
                                     * (robot_1.frame("Left" + surfaceSuffix_1).position().translation()
                                        + robot_1.frame("Right" + surfaceSuffix_1).position().translation()));

  const sva::PTransformd X_0_Fc2 = sva::PTransformd(
      robot_2.posW().rotation(), 0.5
                                     * (robot_2.frame("Left" + surfaceSuffix_2).position().translation()
                                        + robot_2.frame("Right" + surfaceSuffix_2).position().translation()));

  const sva::PTransformd X_b2_fc2 = X_0_Fc2 * robot_2.posW().inv();

  return X_b2_fc2.inv() * X_0_Fc1;
}
