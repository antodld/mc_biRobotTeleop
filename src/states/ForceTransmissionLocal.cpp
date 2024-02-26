#include "ForceTransmissionLocal.h"
#include <mc_joystick_plugin/joystick_inputs.h>

#include "../BiRobotTeleoperation.h"
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/CD/CD_Pair.h>

void ForceTransmissionLocal::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config("force_activation_threshold",force_activation_threshold_);
  config("distance_activation_threshold",distance_activation_threshold_);

  config("deactivation_threshold",deactivation_threshold_);

}

void ForceTransmissionLocal::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  
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
  output("OK");

  if(!active_) //if not active, look if the force sensors measure a contact
  {
    checkActivation(ctl_,1);
    if(!active_)
    {
      checkActivation(ctl_,2);
    }
    return true;
  }

  if(active_force_measurement_ != nullptr)
  {
    double d_b = getContactDistance(ctl_,indx_,limb_a_,limb_b_).norm(); 
    double d_a = getContactDistance(ctl_,indx_ == 1 ? 2 : 1,limb_b_,limb_a_).norm(); 

    active_force_measurement_->update(task_a_->frame().wrench());

    if((active_force_measurement_->eval().vector().norm() < force_activation_threshold_ ||
        d_a > deactivation_threshold_) && !activation_enforced_)
    // if( (d_a > deactivation_threshold_ ) && !activation_enforced_ )
    {
      mc_rtc::log::info("[{}] Contact has been broken deactivate force control\nd_a {} d_b {}",name(),d_a,d_b);
      auto & frame = task_b_->frame();
      ctl.solver().removeTask(task_a_);
      ctl.solver().removeTask(task_b_);
      active_force_measurement_->reset(sva::ForceVecd::Zero());
      active_force_measurement_ = nullptr;
      contact_limb_ = "None";
      indx_ = 0;
      activation_enforced_ = false;
      active_ = false;
      return true;
    }
  }

  sva::ForceVecd measured_wrench_a;
  sva::ForceVecd measured_wrench_b;

  const biRobotTeleop::HumanPose & h_b = ctl.getHumanPose( (indx_ == 1) ? 1 : 0 );
  const biRobotTeleop::HumanPose & h_a = ctl.getHumanPose( (indx_ == 1) ? 0 : 1 );

  const sva::PTransformd X_f_contactF_a = h_b.getOffset(limb_b_) * h_b.getPose(limb_b_) * task_a_->frame().position().inv();
  const sva::PTransformd X_f_contactF_b = h_a.getOffset(limb_a_) * h_a.getPose(limb_a_) * task_b_->frame().position().inv();

  if(task_a_->frame().hasForceSensor())
  {
    measured_wrench_a = task_a_->frame().wrench();
    if(activation_enforced_)
    {
      auto indx = ctl.robot(robot_a_name_).data()->forceSensorsIndex["LeftHandForceSensor"];
      //minus between frame and fs
      ctl.robot(robot_a_name_).data()->forceSensors[indx].wrench(sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d{0,0,20}));
    }
  
  }


  if(task_b_->frame().hasForceSensor() && task_b_->frame().forceSensor().name() != robot_b_custom_force_sensor_name_)
  {
    measured_wrench_b = task_b_->frame().wrench();
  }
  else
  {
    //if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in the mbc at the fb
    
    const mc_rbdyn::Robot & robot_b = ctl.robot(robot_b_name_);
    const auto X_0_fb = ctl.robot(robot_b_name_).posW();

    const auto R_fb_limb_a = sva::PTransformd(h_a.getOffset(limb_a_) * h_a.getPose(limb_a_).rotation()) 
                             * sva::PTransformd(X_0_fb.rotation()).inv();
    const auto X_0_frame = task_b_->frame().position();
    const auto wrench_fb = ctl.getCalibratedExtWrench(ctl.realRobot(robot_b_name_));

    measured_wrench_b = (X_f_contactF_b.inv()
                         * R_fb_limb_a
                         ).dualMul( sva::ForceVecd(Eigen::Vector3d::Zero(),wrench_fb.force()) );
    
    
    // if(activation_enforced_)
    // {
    //   const sva::ForceVecd fake_wrench = 0*sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d{0,-10,-10});
    //   measured_wrench_b = (X_0_frame * X_0_fb.inv()).dualMul( fake_wrench  );
    // }

    const auto fs_indx = robot_b.data()->forceSensorsIndex.at(robot_b_custom_force_sensor_name_);
    robot_b.data()->forceSensors[fs_indx].wrench(measured_wrench_b);

    // task_b_->setMeasuredWrench(measured_wrench_b);
  
  }

  // mc_rtc::log::info("human_a is {}\nX_f_contactF_b trans {}",h_a.name(),X_f_contactF_b.translation());

  const biRobotTeleop::RobotPose & robot_a_pose = indx_ == 1 ? ctl.r_1_ : ctl.r_2_ ;
  const biRobotTeleop::RobotPose & robot_b_pose = indx_ == 2 ? ctl.r_1_ : ctl.r_2_ ;


  const sva::ForceVecd targetWrench_b = (X_f_contactF_b.inv() * robot_a_pose.getOffset(limb_a_)).dualMul(-measured_wrench_a) ;

  const sva::ForceVecd targetWrench_a = (X_f_contactF_a.inv() * robot_b_pose.getOffset(limb_b_)).dualMul(-measured_wrench_b) ;

  task_a_->targetWrench(targetWrench_a);
  task_b_->targetWrench(targetWrench_b);

  

  
  return true;
  
}

bool ForceTransmissionLocal::checkActivation(mc_control::fsm::Controller & ctl_,const int robot_indx)
{
    auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
    const std::string robot_name = "robot_" + std::to_string(robot_indx);
    const biRobotTeleop::HumanPose h_a = robot_indx == 1 ? ctl.hp_1_ : ctl.hp_2_;
    const biRobotTeleop::RobotPose & robot_a_pose = robot_indx == 1 ? ctl.r_1_ : ctl.r_2_;
    const biRobotTeleop::RobotPose & robot_b_pose = robot_indx == 1 ? ctl.r_2_ : ctl.r_1_;
    int filter_indx = 0;
    const std::vector<std::string> fs_limbs = robot_indx == 1 ? force_sensor_limbs_robot_1_ : force_sensor_limbs_robot_2_;
    std::vector<mc_filter::LowPass<sva::ForceVecd>> & activation_force_measurements = robot_indx == 1 ? activation_force_measurements_robot_1_ : activation_force_measurements_robot_2_;
    
    std::vector<bool> & force_activation = robot_indx == 1 ? robot_1_force_activation_ : robot_2_force_activation_;

    for (auto & l :fs_limbs)
    {
      const auto limb = biRobotTeleop::str2Limb(l);
      const auto f = robot_indx == 1 ? ctl.r_1_.getName(limb) : ctl.r_2_.getName(limb);
      const auto w = ctl_.robots().robot(robot_name).frame(f).wrench();
      activation_force_measurements[filter_indx].update(w);

      double min_d = 1e9;
      for(int i = 1 ; i <= biRobotTeleop::Limbs::RightArm ; i++)
      {
        const auto limb_i = static_cast<biRobotTeleop::Limbs>(i);
        const auto frame_b_i = robot_b_pose.getName(limb_i);
        const double d = getContactDistance(ctl_, robot_indx == 1 ? 2 : 1,limb_i,limb).norm();
        min_d = std::min(d,min_d);
      }

      if(activation_force_measurements[filter_indx].eval().vector().norm() > force_activation_threshold_ ||
         min_d < distance_activation_threshold_  || force_activation[filter_indx]) 
      {
        activation_enforced_ = force_activation[filter_indx];
        //Once a force sensor is in contact, we set the limb in contact and activate the force task;
        mc_rtc::log::info("[{}] contact measured on frame {}, adding task",name(),f);
        task_a_= std::make_shared<mc_tasks::force::DampingTask>(ctl.robots().robot(robot_name).frame(f));
        task_a_->load(ctl.solver(),config_(robot_name)("task"));
        task_a_->velFilterGain(0.9);
        task_a_->name(task_a_->name() + "_a");
        limb_a_ = limb;
        ctl.solver().addTask(task_a_);
        active_force_measurement_ = &activation_force_measurements[filter_indx];
        indx_ = robot_indx;
        robot_a_name_ = robot_name;

        robot_b_name_ = (robot_a_name_ == "robot_1") ? "robot_2" : "robot_1";

        limb_b_ = getContactLimb(ctl,robot_indx,limb);
        mc_rbdyn::Robot & robot_b = ctl.robots().robot(robot_b_name_);
        contact_limb_ = biRobotTeleop::limb2Str(limb_b_);
        const std::string link = robot_indx == 1 ? ctl.r_2_.getName(limb_b_) : ctl.r_1_.getName(limb_b_);
        robot_b_custom_force_sensor_name_ = robot_b_name_ + "_" + contact_limb_;
        if(!robot_b.bodyHasForceSensor(link))
        {
          mc_rbdyn::ForceSensor sensor = mc_rbdyn::ForceSensor(robot_b_custom_force_sensor_name_,link,sva::PTransformd::Identity());
          robot_b.addForceSensor(sensor);
        }

        task_b_= std::make_shared<mc_tasks::force::DampingTask>(robot_b.frame(link));
        task_b_->load(ctl.solver(),config_(robot_b_name_)("task"));
        task_b_->velFilterGain(0.9);
        task_b_->name(task_b_->name() + "_b");
        ctl.solver().addTask(task_b_);

        task_robot_1_ = robot_indx == 1 ? task_a_ : task_b_;
        task_robot_2_ = robot_indx == 2 ? task_a_ : task_b_;

        active_ = true;
        mc_rtc::log::info("[{}] robot_a is {} robot_b is {}",name(),robot_a_name_,robot_b_name_);
        mc_rtc::log::info("[{}] task limb_a is {} task limb_b is {}",name(),biRobotTeleop::limb2Str(limb_a_),biRobotTeleop::limb2Str(limb_b_));
        return true;
      }
      filter_indx +=1;
    }

    return false;

}


const biRobotTeleop::Limbs ForceTransmissionLocal::getContactLimb(mc_control::fsm::Controller & ctl_,const int robot_indx,const biRobotTeleop::Limbs & robot_limb) const
{

  biRobotTeleop::Limbs output_limb = biRobotTeleop::Head;
  double min_d = 1e9;

  for (int int_limb = 1 ; int_limb <= biRobotTeleop::Limbs::RightArm ; int_limb++)
  {
    const auto limb = static_cast<biRobotTeleop::Limbs>(int_limb);
    if(limb == biRobotTeleop::Limbs::Pelvis || limb == biRobotTeleop::Limbs::Head){continue;}

    const auto d = getContactDistance(ctl_,robot_indx,robot_limb,limb).norm();

    if(d < min_d)
    {
      output_limb = limb;
      min_d = d;
    }
  }
  mc_rtc::log::info("robot checked is robot_{},min distance is {}",robot_indx , min_d);
  return output_limb;

}


const Eigen::Vector3d ForceTransmissionLocal::getContactDistance(mc_control::fsm::Controller & ctl_,
                                                       const int robot_indx,
                                                       const biRobotTeleop::Limbs limb_robot,
                                                       const biRobotTeleop::Limbs limb_human) const
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const std::string robot_name = "robot_" + std::to_string(robot_indx);
  const auto & robot = ctl.robot(robot_name);
  const auto rp = robot_indx == 1 ? ctl.r_1_ : ctl.r_2_;
  const auto h = ctl.getHumanPose(robot_indx == 1 ? 1 : 0); 

  auto human_cvx = h.getConvex(limb_human);
  const auto & robot_cvx = robot.convex(rp.getConvexName(limb_robot));

  sch::CD_Pair pair_limb_frame(&human_cvx,robot_cvx.second.get());

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  Eigen::Vector3d out;
  out << p1[0] - p2[0],p1[1] - p2[1], p1[2] - p2[2];
  return out;

}


void ForceTransmissionLocal::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

}

void ForceTransmissionLocal::addGUI(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  auto & gui = *ctl_.gui();
  gui.addElement({"States",name()},mc_rtc::gui::Checkbox("Active", [this]() -> bool {return !active_;},[this]() {}));

  size_t count = 0;
  for (auto & f : force_sensor_limbs_robot_1_)
  {
    gui.addElement({"States",name()},
      mc_rtc::gui::ArrayLabel("Measure Wrench robot_1: " + f,{"cx","cy","cz","fx","fy","fz"}, 
                          [this,&ctl,f]() -> Eigen::Vector6d {return ctl.robots().robot("robot_1").frame(ctl.r_1_.getName(biRobotTeleop::str2Limb(f))).wrench().vector() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Label("Activation value robot_1: " + f, 
                        [this,&ctl,count]() -> double {return activation_force_measurements_robot_1_[count].eval().vector().norm() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Checkbox("Force activation robot_1: " + f, 
                        [this,&ctl,count]() -> bool {return robot_1_force_activation_[count] ;},[this,&ctl,count]() {return robot_1_force_activation_[count] = !robot_1_force_activation_[count] ;} ));
    count +=1;
  }
  count = 0;
  for (auto & f : force_sensor_limbs_robot_2_)
  {
    gui.addElement({"States",name()},
      mc_rtc::gui::ArrayLabel("Measure Wrench robot_2: " + f,{"cx","cy","cz","fx","fy","fz"}, 
                          [this,&ctl,f]() -> Eigen::Vector6d {return ctl.robots().robot("robot_2").frame(ctl.r_2_.getName(biRobotTeleop::str2Limb(f))).wrench().vector() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Label("Activation value robot_2: " + f, 
                        [this,&ctl,count]() -> double {return activation_force_measurements_robot_2_[count].eval().vector().norm() ;} ));
    gui.addElement({"States",name()},
    mc_rtc::gui::Checkbox("Force activation robot_2: " + f, 
                        [this,&ctl,count]() -> bool {return robot_2_force_activation_[count] ;},[this,&ctl,count]() {return robot_2_force_activation_[count] = !robot_2_force_activation_[count] ;} ));
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
