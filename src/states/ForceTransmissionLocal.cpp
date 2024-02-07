#include "ForceTransmissionLocal.h"
#include <mc_joystick_plugin/joystick_inputs.h>

#include "../BiRobotTeleoperation.h"
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/CD/CD_Pair.h>

void ForceTransmissionLocal::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config("activation_threshold",activation_threshold_);
  config("deactivation_threshold",deactivation_threshold_);

}

void ForceTransmissionLocal::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  
  robot_1_pose_ = ctl.r_1_;
  robot_2_pose_ = ctl.r_2_;

  dt_ = ctl_.timeStep;

  config_("robot_1")("force_sensor_limbs",force_sensor_limbs_robot_1_);
  config_("robot_2")("force_sensor_limbs",force_sensor_limbs_robot_2_);

  for (int _ = 0 ; _ < force_sensor_limbs_robot_1_.size(); _++)
  {
    activation_force_measurements_robot_1_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_,0.5));
    robot_1_force_activation_.push_back(false);
  }
  for (int _ = 0 ; _ < force_sensor_limbs_robot_2_.size(); _++)
  {
    activation_force_measurements_robot_2_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_,0.5));
    robot_2_force_activation_.push_back(false);
  }

  dt_ = ctl.timeStep;


  addGUI(ctl_);
  addLog(ctl_);

}

bool ForceTransmissionLocal::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  if(!active_) //if not active, look if the force sensors measure a contact
  {
    checkActivation(ctl_,1);
    if(!active_)
    {
      checkActivation(ctl_,2);
    }
  }

  if(active_)
  {
    
    if(active_force_measurement_ != nullptr)
    {
      double d = getContactLimbDistance(ctl_,indx_,task_a_->frame().name(),biRobotTeleop::str2Limb(contact_limb_)); 
      active_force_measurement_->update(task_a_->frame().wrench());

      if(active_force_measurement_->eval().vector().norm() < activation_threshold_ ||
          d > deactivation_threshold_)
      {
        mc_rtc::log::info("[{}] Contact has been broken deactivate force control",name());
        auto & frame = task_b_->frame();
        ctl.solver().removeTask(task_a_);
        ctl.solver().removeTask(task_b_);
        active_force_measurement_->reset(sva::ForceVecd::Zero());
        active_force_measurement_ = nullptr;
        contact_limb_ = "None";
        indx_ = 0;
        active_ = false;
        return true;
      }
    }

    sva::ForceVecd measured_wrench_a;
    sva::ForceVecd measured_wrench_b;

    if(task_a_->frame().hasForceSensor()){measured_wrench_a = task_a_->frame().wrench();}
    else
    {
      //if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in the mbc at the fb
      const mc_rbdyn::Robot & robot_a = ctl.robots().robot("robot_" + std::to_string(indx_));
      const auto X_0_fb = robot_a.posW();
      const auto X_0_frame = task_a_->frame().position();
      measured_wrench_a = (X_0_frame * X_0_fb.inv()).dualMul(robot_a.mbc().force[0]);
      task_a_->setMeasuredWrench(measured_wrench_a);
  
    }
    if(task_b_->frame().hasForceSensor()){measured_wrench_b = task_b_->frame().wrench();}
    else
    {
      //if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in the mbc at the fb
      const mc_rbdyn::Robot & robot_b = ctl.robots().robot("robot_" + std::to_string( (indx_%2) + 1));
      const auto X_0_fb = robot_b.posW();
      const auto X_0_frame = task_b_->frame().position();
      measured_wrench_b = (X_0_frame * X_0_fb.inv()).dualMul(robot_b.mbc().force[0]);
      task_b_->setMeasuredWrench(measured_wrench_b);
    
    }
  
    const biRobotTeleop::HumanPose & h_b = ctl.getHumanPose(indx_%2);
    const biRobotTeleop::HumanPose & h_a = ctl.getHumanPose( (indx_ + 1)%2 );

    X_f_contactF_a = h_b.getOffset(limb_b_) * h_b.getPose(limb_b_) * task_a_->frame().position().inv();
    X_f_contactF_b = h_a.getOffset(limb_a_) * h_a.getPose(limb_a_) * task_b_->frame().position().inv();


    const biRobotTeleop::RobotPose robot_b_pose = indx_ == 2 ? robot_2_pose_ : robot_1_pose_;
    const biRobotTeleop::RobotPose robot_a_pose = indx_ == 1 ? robot_1_pose_ : robot_2_pose_;

    sva::ForceVecd targetWrench_b = (X_f_contactF_b.inv() * robot_a_pose.getOffset(limb_a_)).dualMul(-measured_wrench_a) ;

    sva::ForceVecd targetWrench_a = (X_f_contactF_a.inv() * robot_b_pose.getOffset(limb_b_)).dualMul(-measured_wrench_b) ;

    task_a_->targetWrench(targetWrench_a);
    task_b_->targetWrench(targetWrench_b);

    
    // task_robot_2_->targetWrench( target_force + sva::ForceVecd(r_d_.cwiseProduct(vel_.vector())));
    // task_robot_1_->targetPose(task_robot_1_->frame().position());
    // count++;
  
  }
  
  output("OK");
  return true;
  
}

bool ForceTransmissionLocal::checkActivation(mc_control::fsm::Controller & ctl_,const int robot_indx)
{
    auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
    const std::string robot_name = "robot_" + std::to_string(robot_indx);
    const biRobotTeleop::RobotPose & robot_a_pose = robot_indx == 1 ? robot_1_pose_ : robot_2_pose_;
    const biRobotTeleop::RobotPose & robot_b_pose = robot_indx == 1 ? robot_2_pose_ : robot_1_pose_;
    int filter_indx = 0;
    const std::vector<std::string> fs_limbs = robot_indx == 1 ? force_sensor_limbs_robot_1_ : force_sensor_limbs_robot_2_;
    std::vector<mc_filter::LowPass<sva::ForceVecd>> & activation_force_measurements = robot_indx == 1 ? activation_force_measurements_robot_1_ : activation_force_measurements_robot_2_;
    
    std::vector<bool> & force_activation = robot_indx == 1 ? robot_1_force_activation_ : robot_2_force_activation_;

    for (auto & l :fs_limbs)
    {
      const auto limb = biRobotTeleop::str2Limb(l);
      const auto f = robot_indx == 1 ? robot_1_pose_.getName(limb) : robot_2_pose_.getName(limb);
      const auto w = ctl_.robots().robot(robot_name).frame(f).wrench();
      activation_force_measurements[filter_indx].update(w);
      if(activation_force_measurements[filter_indx].eval().vector().norm() > activation_threshold_ || force_activation[filter_indx]) 
      {
        //Once a force sensor is in contact, we set the limb in contact and activate the force task;
        mc_rtc::log::info("[{}] force measured on frame {}, adding task",name(),f);
        task_a_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robots().robot(robot_name).frame(f));
        task_a_->load(ctl.solver(),config_(robot_name)("task"));
        task_a_->velFilterGain(0);
        limb_a_ = limb;
        ctl.solver().addTask(task_a_);
        active_force_measurement_ = &activation_force_measurements[filter_indx];

        indx_ = robot_indx;

        limb_b_ = getContactLimb(ctl,robot_indx,f);
        std::string robot_b_name = indx_ == 1 ? "robot_2" : "robot_1";
        mc_rbdyn::Robot & robot_b = ctl.robots().robot(robot_b_name);
        contact_limb_ = biRobotTeleop::limb2Str(limb_b_);
        const std::string link = robot_indx == 1 ? robot_2_pose_.getName(limb_b_) : robot_1_pose_.getName(limb_b_);
        const Eigen::Vector3d robot_b_contact_pose = getContactPose(ctl_, indx_ == 1 ? 2 : 1,limb_b_,limb_a_); 
        const auto X_0_contactF = sva::PTransformd(robot_b.frame(link).position().rotation(),robot_b_contact_pose);

        task_b_= std::make_shared<mc_tasks::force::AdmittanceTask>(robot_b.frame(link));
        task_b_->load(ctl.solver(),config_(robot_b_name)("task"));
        task_b_->velFilterGain(0);
        ctl.solver().addTask(task_b_);

        task_robot_1_ = robot_indx == 1 ? task_a_ : task_b_;
        task_robot_2_ = robot_indx == 1 ? task_b_ : task_a_;

        active_ = true;
        return true;
      }
      filter_indx +=1;
    }

    return false;

}


const biRobotTeleop::Limbs ForceTransmissionLocal::getContactLimb(mc_control::fsm::Controller & ctl_,const int robot_indx,const std::string & frame) const
{

  biRobotTeleop::Limbs output_limb = biRobotTeleop::Head;
  double min_d = 1e9;

  for (int int_limb = 1 ; int_limb <= biRobotTeleop::Limbs::RightArm ; int_limb++)
  {
    const auto limb = static_cast<biRobotTeleop::Limbs>(int_limb);

    auto d = getContactLimbDistance(ctl_,robot_indx,frame,limb);

    if(d < min_d)
    {
      output_limb = limb;
      min_d = d;
    }
  }
  return output_limb;

}

double ForceTransmissionLocal::getContactLimbDistance(mc_control::fsm::Controller & ctl_,const int robot_indx,const std::string & frame,const biRobotTeleop::Limbs limb) const
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  
  const sva::PTransformd & X_0_frame = ctl_.robots().robot("robot_" + std::to_string(robot_indx)).frame(frame).position();
  const auto p_frame = X_0_frame.translation();
  const auto & h = ctl.getHumanPose(robot_indx % 2);


  sch::S_Sphere sphere(0.01);
  sphere.setPosition(p_frame.x(),p_frame.y(),p_frame.z());
  
  auto cvx = h.getConvex(limb);
  
  sch::CD_Pair pair_limb_frame(&cvx,&sphere);

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  return (p1 - p2).norm();

}

Eigen::Vector3d ForceTransmissionLocal::getContactPose(mc_control::fsm::Controller & ctl_,
                                                       const int robot_indx,
                                                       const biRobotTeleop::Limbs limb_robot,
                                                       const biRobotTeleop::Limbs limb_human)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const std::string robot_name = "robot_" + std::to_string(robot_indx);
  auto & robot = ctl.robots().robot(robot_name);

  const auto & robot_b_pose = robot_indx == 1 ? robot_1_pose_ : robot_2_pose_;
  const auto h = ctl.getHumanPose(robot_indx%2); //human in front of robot_b

  auto human_cvx = h.getConvex(limb_human);
  auto robot_cvx = robot.convex(robot_b_pose.getConvexName(limb_robot));

  sch::CD_Pair pair_limb_frame(&human_cvx,robot_cvx.second.get());

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  Eigen::Vector3d out;
  out << p2[0],p2[1],p2[2];
  return out;

}


void ForceTransmissionLocal::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

}

void ForceTransmissionLocal::addGUI(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();
  gui.addElement({"States",name()},mc_rtc::gui::Checkbox("Active", [this]() -> bool {return !active_;},[this]() {}));

  size_t count = 0;
  for (auto & f : force_sensor_limbs_robot_1_)
  {
    gui.addElement({"States",name()},
      mc_rtc::gui::ArrayLabel("Measure Wrench robot_1: " + f,{"cx","cy","cz","fx","fy","fz"}, 
                          [this,&ctl_,f]() -> Eigen::Vector6d {return ctl_.robots().robot("robot_1").frame(robot_1_pose_.getName(biRobotTeleop::str2Limb(f))).wrench().vector() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Label("Activation value robot_1: " + f, 
                        [this,&ctl_,count]() -> double {return activation_force_measurements_robot_1_[count].eval().vector().norm() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Checkbox("Force activation robot_1: " + f, 
                        [this,&ctl_,count]() -> bool {return robot_1_force_activation_[count] ;},[this,&ctl_,count]() {return robot_1_force_activation_[count] = true ;} ));
    count +=1;
  }
  count = 0;
  for (auto & f : force_sensor_limbs_robot_2_)
  {
    gui.addElement({"States",name()},
      mc_rtc::gui::ArrayLabel("Measure Wrench robot_2: " + f,{"cx","cy","cz","fx","fy","fz"}, 
                          [this,&ctl_,f]() -> Eigen::Vector6d {return ctl_.robots().robot("robot_2").frame(robot_2_pose_.getName(biRobotTeleop::str2Limb(f))).wrench().vector() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Label("Activation value robot_2: " + f, 
                        [this,&ctl_,count]() -> double {return activation_force_measurements_robot_2_[count].eval().vector().norm() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Checkbox("Force activation robot_2: " + f, 
                        [this,&ctl_,count]() -> bool {return robot_2_force_activation_[count] ;},[this,&ctl_,count]() {return robot_2_force_activation_[count] = true ;} ));
    count +=1; 
  }

}

void ForceTransmissionLocal::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl.gui()->removeCategory({"States",name()});
  ctl_.solver().removeTask(task_a_);
  ctl_.solver().removeTask(task_b_);

}

EXPORT_SINGLE_STATE("ForceTransmissionLocal", ForceTransmissionLocal)
