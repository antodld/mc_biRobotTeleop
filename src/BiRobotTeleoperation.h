#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/MCController.h>
#include <mc_control/fsm/Controller.h>
#include "api.h"
#include <biRobotTeleop/HumanRobotPose.h>
#include <ros/ros.h>
#include <mc_rtc_ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <mc_rbdyn/RobotLoader.h>
#include <biRobotTeleop/HumanRobotDataReceiver.h>
#include <mc_control/ControllerServer.h>
#include <mc_rtc/gui.h>
#include <mc_joystick_plugin/joystick_inputs.h>

struct BiRobotTeleoperation_DLLAPI BiRobotTeleoperation : public mc_control::fsm::Controller
{
  BiRobotTeleoperation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  mc_tasks::MetaTask* getTask(const std::string name)
  {
    for (auto & t : solver().tasks())
    {
      if(t->name() == name)
      {
        return t;
      }
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("tasks {} is not in the solver",name);
  }

  void create_collision_cstr(const mc_rtc::Configuration & config);


  void updateHumanLink(const mc_rbdyn::Robot & human,const std::string & link ,biRobotTeleop::HumanPose & human_pose,const biRobotTeleop::Limbs human_link)
  {
    Eigen::Vector3d noise_pose = 0.00 * Eigen::Vector3d::Random();
    human_pose.setPose(human_link,sva::PTransformd(noise_pose) * human.bodyPosW(link));
    const sva::PTransformd X_link_link0 = sva::PTransformd( (human_pose.getPose(human_link).inv()).rotation(),Eigen::Vector3d::Zero() );
    
    auto noise = 0.0 * Eigen::Vector6d::Random();

    human_pose.setVel(human_link,X_link_link0 * human.bodyVelB(link) + sva::MotionVecd(noise));

    // const sva::MotionVecd v = human_pose.getVel(human_link);
    // human_pose.setAcc(human_link, X_link_link0  * human.bodyAccB(link) + sva::MotionVecd(Eigen::Vector3d::Zero(),v.angular().cross(v.linear())));
    human_pose.setLimbActiveState(human_link,true);
  }
  void updateHumanPose(const mc_rbdyn::Robot & human ,biRobotTeleop::HumanPose & human_pose)
  {
    updateHumanLink(human,"LHandLink",human_pose,biRobotTeleop::Limbs::LeftHand);
    updateHumanLink(human,"RHandLink",human_pose,biRobotTeleop::Limbs::RightHand);
    updateHumanLink(human,"LForearmLink",human_pose,biRobotTeleop::Limbs::LeftForearm);
    updateHumanLink(human,"RForearmLink",human_pose,biRobotTeleop::Limbs::RightForearm);
    updateHumanLink(human,"LArmLink",human_pose,biRobotTeleop::Limbs::LeftArm);
    updateHumanLink(human,"RArmLink",human_pose,biRobotTeleop::Limbs::RightArm);
    updateHumanLink(human,"HipsLink",human_pose,biRobotTeleop::Limbs::Pelvis);

  }

  /**
   * @brief Update the HumanPose object
   * 
   * @param h HumanPose Object entry data
   * @param h_target  HumanPose object that will be updated
   */
  void updateHumanPose(const biRobotTeleop::HumanPose & h, biRobotTeleop::HumanPose & h_target)
  {
    for(int i = 0 ; i <= biRobotTeleop::RightArm ; i++)
    {
      const auto limb = static_cast<biRobotTeleop::Limbs>(i);
      h_target.setPose(limb,h.getPose(limb));
      h_target.setVel(limb,h.getVel(limb));
      h_target.setAcc(limb,h.getAcc(limb));
      h_target.setLimbActiveState(limb,h.limbActive(limb));
    }
  }

  void resetToRealRobot(const std::string & name);
  

  void updateDistantHumanRobot();

  biRobotTeleop::HumanPose hp_1_;
  biRobotTeleop::HumanPose hp_2_;
  biRobotTeleop::RobotPose r_1_;
  biRobotTeleop::RobotPose r_2_;
  biRobotTeleop::HumanPose hp_1_filtered_;
  biRobotTeleop::HumanPose hp_2_filtered_;
  mc_rbdyn::RobotsPtr external_robots_ = nullptr;
  biRobotTeleop::HumanRobotDataReceiver hp_rec_;

  const int getDistantHumanIndx() const noexcept
  {
    return distant_human_indx_;
  }

  const std::string getDistantRobotName() const noexcept
  {
    return robots().robot(getDistantHumanIndx()).name();
  }


  /**
   * @brief Get the index of the human in front of the mainRobot of this controller 
   * 
   * @return const int 
   */
  const int getHumanIndx() const noexcept
  {
    return (distant_human_indx_ == 0) ? 1 : 0;
  }

  biRobotTeleop::HumanPose & getHumanPose(const int indx,const bool filtered = false)
  {
    if(!filtered)
    {
      return (indx == 0) ? hp_1_ : hp_2_;
    }
    {
      return (indx == 0) ? hp_1_filtered_ : hp_2_filtered_;
    }
  }

  const biRobotTeleop::RobotPose & getRobotPose(const int indx) const
  {
      return (indx == 0) ? r_1_ : r_2_;
  }

  const mc_rtc::Configuration & getGlobalConfig() const noexcept
  {
    return global_config_;
  }

  mc_rtc::gui::StateBuilder & getGUIBuilder()
  {
    return gui_builder_;
  }

  const bool useRos() const noexcept
  {
    return use_ros_;
  }

  bool joystickButtonPressed(const joystickButtonInputs input);

  void hardEmergency()
  {
    emergency_ = true;
    mc_rtc::log::critical("Hard Emergency called");
  }

  size_t ctl_count() const noexcept
  {
    return ctl_count_;
  }

  const sva::ForceVecd getCalibratedExtWrench(const mc_rbdyn::Robot & robot) const
  {
    assert(external_wrench_calib_.size() > 1 );
    auto & ext_wrench = robot.mbc().force.at(0);

    auto & off = robot.name() == "robot_1" ? external_wrench_calib_[0] : external_wrench_calib_[1];
    return ext_wrench - off;
  }

  void CalibrateExtWrench(const mc_rbdyn::Robot & robot)
  {
    assert(external_wrench_calib_.size() > 1 );
    auto & ext_wrench = robot.mbc().force.at(0);
    if(robot.name() == "robot_1")
    {
      external_wrench_calib_[0] = ext_wrench;
    }
    else
    {
      external_wrench_calib_[1] = ext_wrench;
    }
    mc_rtc::log::info("External wrench calibrated on {} at {}",ext_wrench,robot.name());

  }

  sva::ForceVecd getExtWrenchGT(const mc_rbdyn::Robot & robot,const std::string & frame)
  {
    const auto & X_0_fb = robot.posW();
    const auto & X_0_lh = robot.frame(frame).position();
    const auto w_lh = robot.frame(frame).wrench();

    return (X_0_fb * X_0_lh.inv()).dualMul(w_lh);

  }

  std::vector<sva::ForceVecd> external_wrench_calib_;


private:
  
  std::unique_ptr<mc_control::ControllerServer> server_;
  mc_rtc::gui::StateBuilder gui_builder_;
  bool emergency_ = false;
  double cl_gain_ = 1e-6;

  //distant_controller_data
  std::string ip_ = "localhost"; 
  int sub_port_ = 4242;
  int pub_port_ = 4343;
  std::string distant_human_name_ = "human_1";
  std::string distant_robot_name_ = "robot_2";
  int distant_human_indx_ = 0;
  std::string local_human_name_ = "human_2";

  mc_rtc::Configuration global_config_;

  ros::Publisher sch_pub_;
  visualization_msgs::MarkerArray markers_;

  bool useFilteredData_ = false;

  bool use_ros_ = false;

  size_t ctl_count_ = 0;

  void addReplayLog(const int indx);

};

/**
 * @brief Set robot_2 fb such as they have the same foot center 
 * 
 * @param robot_1 
 * @param robot_2 
 * @return sva::PTransformd 
 */
sva::PTransformd alignFeet(const mc_rbdyn::Robot & robot_1,const std::string & surfaceSuffix_1, const mc_rbdyn::Robot & robot_2,const std::string & surfaceSuffix_2);