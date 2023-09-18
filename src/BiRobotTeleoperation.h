#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/MCController.h>
#include <mc_control/fsm/Controller.h>
#include "api.h"
#include <biRobotTeleop/HumanRobotPose.h>
#include <ros/ros.h>
#include <mc_rtc_ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

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

  void updateHumanLink(const mc_rbdyn::Robot & human,const std::string & link ,biRobotTeleop::HumanPose & human_pose,const biRobotTeleop::Limbs human_link)
  {
    human_pose.setPose(human_link,human.bodyPosW(link));
    sva::PTransformd X_link_link0 = sva::PTransformd( (human_pose.getPose(human_link).inv()).rotation(),Eigen::Vector3d::Zero() ); 
    human_pose.setVel(human_link,X_link_link0 * human.bodyVelB(link));
    human_pose.setAcc(human_link, X_link_link0  * human.bodyAccB(link));
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

private:
  
  mc_rbdyn::RobotsPtr external_robots_ = nullptr;
  
  ros::Publisher sch_pub_;
  visualization_msgs::MarkerArray markers_;

};

/**
 * @brief Set robot_2 fb such as they have the same foot center 
 * 
 * @param robot_1 
 * @param robot_2 
 * @return sva::PTransformd 
 */
sva::PTransformd alignFeet(const mc_rbdyn::Robot & robot_1,const std::string & surfaceSuffix_1, const mc_rbdyn::Robot & robot_2,const std::string & surfaceSuffix_2);