#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/biRobotTeleopTask.h>
#include <bilateralteleop/HumanRobotPose.h>
#include <bilateralTeleop_dataLink/type.h>

struct BiRobotTeleoperation_Task : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void updateHumanLink(const mc_rbdyn::Robot & human,const std::string & link ,bilateralTeleop::HumanPose & human_pose,const bilateralTeleop::Limbs human_link)
  {
    human_pose.setPose(human_link,human.bodyPosW(link));
    sva::PTransformd X_link_link0 = sva::PTransformd( (human_pose.getPose(human_link).inv()).rotation(),Eigen::Vector3d::Zero() ); 
    human_pose.setVel(human_link,X_link_link0 * human.bodyVelB(link));
    human_pose.setAcc(human_link, X_link_link0  * human.bodyAccB(link));
  }
  void updateHumanPose(const mc_rbdyn::Robot & human ,bilateralTeleop::HumanPose & human_pose)
  {
    updateHumanLink(human,"LHandLink",human_pose,bilateralTeleop::Limbs::LeftHand);
    updateHumanLink(human,"RHandLink",human_pose,bilateralTeleop::Limbs::RightHand);
    updateHumanLink(human,"LForearmLink",human_pose,bilateralTeleop::Limbs::LeftForearm);
    updateHumanLink(human,"RForearmLink",human_pose,bilateralTeleop::Limbs::RightForearm);
    updateHumanLink(human,"LArmLink",human_pose,bilateralTeleop::Limbs::LeftArm);
    updateHumanLink(human,"RArmLink",human_pose,bilateralTeleop::Limbs::RightArm);
    updateHumanLink(human,"HipsLink",human_pose,bilateralTeleop::Limbs::Pelvis);

  }

  std::shared_ptr<mc_tasks::biRobotTeleopTask> biTask_;

  std::string r1_name_ = "";
  std::string r2_name_ = "";

  mc_rtc::Configuration state_config_;

  //The weight is a linear interpolation clamped in the distance range between the weight range
  Eigen::Vector2d weightRange_ = {20,1000};
  Eigen::Vector2d weightDistanceRange_ = Eigen::Vector2d::Zero();

};
