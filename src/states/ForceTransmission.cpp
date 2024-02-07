#include "ForceTransmission.h"
#include <mc_joystick_plugin/joystick_inputs.h>

#include "../BiRobotTeleoperation.h"
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/CD/CD_Pair.h>

void ForceTransmission::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config_("delay",t_delay_);
  config("integration_window",int_window_);
  config("activation_threshold",activation_threshold_);
  config("deactivation_threshold",deactivation_threshold_);

}

void ForceTransmission::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  std::string robot_name = ctl_.robots().robot(0).name();
  

  dt_ = ctl_.timeStep;

  config_(robot_name)("force_sensor_limbs",force_sensor_limbs_);
  for (int _ = 0 ; _ < force_sensor_limbs_.size(); _++)
  {
    activation_force_measurements_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_,1));
  }

  const int robot_indx = ctl_.robots().robotIndex(robot_name);
  robotPose_ = ctl.getRobotPose(robot_indx);


  const mc_rbdyn::Robot & robot = ctl_.robots().robot(robot_indx);
  dt_ = ctl.timeStep;


  addGUI(ctl_);
  addLog(ctl_);

  ctl.getGUIBuilder().addElement({"BiRobotTeleop","ForceTransmission"},mc_rtc::gui::ArrayLabel("e_r_in",[this]() -> const Eigen::VectorXd {return e_r_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("e_s_in",[this]() -> const Eigen::VectorXd {return e_s_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("Measured Force",[this]() -> const Eigen::VectorXd {return measured_force_.vector();} ),
                                                              mc_rtc::gui::Label("Contact Limb",[this]() -> const std::string & {return contact_limb_;} ),
                                                              mc_rtc::gui::Label("Robot Limb",[this]() -> const std::string & {return robot_limb_;} ));

  ctl.hp_rec_.subsbscribe("e_r_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_r_in");
  ctl.hp_rec_.subsbscribe("e_s_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_s_in");
  ctl.hp_rec_.subsbscribe("Measured Force",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"Measured Force");
  ctl.hp_rec_.subsbscribe("Contact Limb",mc_rtc::gui::Elements::Label,{"BiRobotTeleop","ForceTransmission"},"Contact Limb");
  ctl.hp_rec_.subsbscribe("Robot Limb",mc_rtc::gui::Elements::Label,{"BiRobotTeleop","ForceTransmission"},"Robot Limb");


}

bool ForceTransmission::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  
  std::string distant_contact_limb = "None"; //is also the human link that is locally force controlled by the robot
  std::string distant_robot_limb = "None"; //The distant robot link that is force controlled it is the image of the local human 
  ctl.hp_rec_.getSubscribedData<std::string>(distant_contact_limb,"Contact Limb");
  ctl.hp_rec_.getSubscribedData<std::string>(distant_robot_limb,"Robot Limb");


  if(!active_) //if not active, look if the force sensors measure a contact
  {
    int filter_indx = 0;
    for (auto & l :force_sensor_limbs_)
    {
      const auto limb = biRobotTeleop::str2Limb(l);
      const auto frame_name = robotPose_.getName(limb);
      const auto w = ctl_.robot().frame(frame_name).wrench();
      activation_force_measurements_[filter_indx].update(w);
      if(activation_force_measurements_[filter_indx].eval().vector().norm() > activation_threshold_) 
      {
        //Once a force sensor is in contact, we set the limb in contact and activate the force task;
        mc_rtc::log::info("[{}] force measured on frame {}, adding task",name(),frame_name);
        const auto contact_limb = getContactLimb(ctl,frame_name);
        contact_limb_ = biRobotTeleop::limb2Str(contact_limb);
        robot_limb_ = limb;
        task_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot().frame(frame_name));
        task_->load(ctl.solver(),config_("task"));
        task_->velFilterGain(0);
        ctl.solver().addTask(task_);
        active_force_measurement_ = &activation_force_measurements_[filter_indx];
        active_ = true;
        break;
      }
      filter_indx +=1;
    }
    if(distant_contact_limb != "None")
    {
      //The distant robot is applying force on the contact limb
      robot_limb_ = distant_contact_limb;
      biRobotTeleop::Limbs human_limb = biRobotTeleop::str2Limb(distant_robot_limb);
      biRobotTeleop::Limbs robot_limb = biRobotTeleop::str2Limb(robot_limb_);

      mc_rtc::log::info("[{}] force measured on limb {}, adding corresponding task",name(),distant_contact_limb);
      const std::string frameName = robotPose_.getName( biRobotTeleop::str2Limb(robot_limb_));  
      const auto contactPose = getContactPose(ctl_,robot_limb,human_limb);
      const auto X_0_contactF = sva::PTransformd(ctl.robot().frame(frameName).position().rotation(),contactPose);
      const auto X_f_contactF = X_0_contactF * ctl.robot().frame(frameName).position().inv();

      auto & robot_frame = ctl.robot().makeFrame("contact_frame",ctl.robot().frame(frameName),X_f_contactF,false);

      task_= std::make_shared<mc_tasks::force::AdmittanceTask>(robot_frame);
      task_->load(ctl.solver(),config_("task"));  
      task_->velFilterGain(0);
      ctl.solver().addTask(task_);
      active_ = true;
    } 
  }

  if(active_)
  {
    const auto & h = ctl.getHumanPose(ctl.getHumanIndx());


    //R rotate the task frame to the unified frame ori
    const Eigen::Matrix3d R_taskFrame_RobotLimb = robotPose_.getOffset( biRobotTeleop::str2Limb(robot_limb_)).rotation()* task_->frame().X_b_f().rotation().transpose();
    const Eigen::Matrix3d R_taskFrame_HumanLimb = h.getPose(biRobotTeleop::str2Limb(distant_robot_limb)).rotation() 
                                                  * task_->frame().position().rotation().transpose();
    const Eigen::Matrix3d R_HumanLimb_RobotLimb = R_taskFrame_RobotLimb * R_taskFrame_HumanLimb.transpose();


    const sva::PTransformd X_0_frame = task_->frame().position();
    const sva::MotionVecd prev_vel = vel_;
    vel_ = sva::PTransformd(R_taskFrame_RobotLimb) * sva::PTransformd(X_0_frame.rotation()) * task_->frame().velocity();

    
    if(active_force_measurement_ != nullptr)
    {
      double d = getContactLimbDistance(ctl_,task_->frame().name(),biRobotTeleop::str2Limb(contact_limb_)); 
      active_force_measurement_->update(task_->frame().wrench());

      if(active_force_measurement_->eval().vector().norm() < activation_threshold_ ||
          d > deactivation_threshold_)
      {
        mc_rtc::log::info("[{}] Contact has been broken deactivate force control",name());
        ctl.solver().removeTask(task_);
        active_force_measurement_->reset(sva::ForceVecd::Zero());
        active_force_measurement_ = nullptr;
        contact_limb_ = "None";
        resetEnergyState();
        active_ = false;
        return done_;
      }


    }
    else if(distant_contact_limb == "None")
    {
        mc_rtc::log::info("[{}] Contact has been broken deactivate force control",name());
        ctl.solver().removeTask(task_);
        active_ = false;    
        resetEnergyState();  
        return done_;
    }

    sva::ForceVecd measured_force_frame; //measured force in task frame;

    if(task_->frame().hasForceSensor())
    {
      measured_force_frame = task_->frame().wrench();    
    }
    else
    {
      //if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in the mbc at the fb
      const auto X_0_fb = ctl.robot().posW();
      const auto X_0_f = task_->frame().position();
      const auto X_0_b = task_->frame().X_b_f().inv() * X_0_f;

      measured_force_frame = (X_0_f * X_0_fb.inv()).dualMul(  ctl.robot().mbc().force[0] );
    
    }
    measured_force_ = sva::PTransformd(R_taskFrame_RobotLimb).dualMul(measured_force_frame);



    Eigen::VectorXd received_force = Eigen::VectorXd::Zero(6);
    ctl.hp_rec_.getSubscribedData<Eigen::VectorXd>(received_force,"Measured Force");
    
    Eigen::VectorXd received_e_r_in = Eigen::VectorXd::Zero(6);
    ctl.hp_rec_.getSubscribedData<Eigen::VectorXd>(received_e_r_in,"e_r_in");
  
    Eigen::VectorXd received_e_s_in = Eigen::VectorXd::Zero(6);
    ctl.hp_rec_.getSubscribedData<Eigen::VectorXd>(received_e_s_in,"e_s_in");
    
    received_force_.push_back( sva::ForceVecd( received_force.segment(0,6)) );
    if(received_force_.size() > max_buffer_size_)
    {
      received_force_.erase(received_force_.begin());
    }
    
    received_e_r_in_.push_back(received_e_r_in.segment(0,6));
    if(received_e_r_in_.size() > max_buffer_size_)
    {
      received_e_r_in_.erase(received_e_r_in_.begin());
    }
    
    received_e_s_in_.push_back(received_e_s_in.segment(0,6));
    if(received_e_s_in_.size() > max_buffer_size_)
    {
      received_e_s_in_.erase(received_e_s_in_.begin());
    }
    

    if (!stop_)
    {
      if(task_->dimWeight().norm() == 0)
      {
        task_->dimWeight(weight_);
      }
      else
      {
        weight_ = task_->dimWeight();
      }
    }
    else
    {
      task_->dimWeight(Eigen::VectorXd::Zero(6));
    }

    //We target minus the measured force on the distant robot, locally, the distant robot coincide with the local human pose
    //target force is expressed in task frame
    const sva::ForceVecd target_force = sva::PTransformd(R_HumanLimb_RobotLimb).dualMul( (-getReceivedData<sva::ForceVecd>(received_force_)) );

    //Energy is locally expressed in task frame
    updateEnergyState(e_r_out_, -target_force,vel_);
    updateEnergyWindow(e_r_out_,E_r_out_);
    
    updateEnergyState(e_s_out_,-measured_force_,vel_);
    updateEnergyWindow(e_s_out_,E_s_out_);
    
    updateEnergyState(e_r_in_,target_force,vel_);
    updateEnergyWindow(e_r_in_,E_r_in_);
    
    updateEnergyState(e_s_in_,measured_force_,vel_);
    updateEnergyWindow(e_s_in_,E_s_in_);


    e_d_ += r_d_.cwiseProduct( prev_vel.vector().cwiseProduct(prev_vel.vector())) * dt_; //P_d = R * v^2
    updateEnergyWindow(e_d_,E_d_);


    e_obs_ =    sva::PTransformd(R_HumanLimb_RobotLimb).matrix() * getReceivedData<Eigen::Vector6d>(received_e_r_in_) 
              + sva::PTransformd(R_HumanLimb_RobotLimb).matrix() * getReceivedData<Eigen::Vector6d>(received_e_s_in_) 
              + E_d_.back() - E_d_.front() 
              - (E_s_out_.back() - E_s_out_.front()) 
              - (E_r_out_.back() - E_r_out_.front());
    for (size_t i = 0 ; i < 6 ; i++ )
    {
      if(e_obs_(i) < 0 && use_load_)
      {
        r_d_(i) += 10;
      }
      else
      {
        r_d_(i) = 0;
      }
    }  
    // mc_rtc::log::info("[{}], target force {}",name(),target_force.force().z());
    task_->targetWrench( sva::PTransformd(R_taskFrame_RobotLimb).transMul( (target_force + sva::ForceVecd(r_d_.cwiseProduct(vel_.vector())))) );
    task_->targetPose(X_0_frame);
    count++;
  
  }

  output("OK");
  return true;
}

void ForceTransmission::updateEnergyState(Eigen::Vector6d & energy,const sva::ForceVecd & force, const sva::MotionVecd vel)
{
  Eigen::Vector6d p = force.vector().cwiseProduct(vel.vector());
  for (size_t i = 0 ; i < 6 ; i++ )
  {
    if(p(i) >= 0)
    {
      energy(i) += p(i) * dt_;
    }
  } 

}

const biRobotTeleop::Limbs ForceTransmission::getContactLimb(mc_control::fsm::Controller & ctl_,const std::string & frame) const
{

  biRobotTeleop::Limbs output_limb = biRobotTeleop::Head;
  double min_d = 1e9;

  for (int int_limb = 1 ; int_limb <= biRobotTeleop::Limbs::RightArm ; int_limb++)
  {
    const auto limb = static_cast<biRobotTeleop::Limbs>(int_limb);

    auto d = getContactLimbDistance(ctl_,frame,limb);

    if(d < min_d)
    {
      output_limb = limb;
      min_d = d;
    }
  }
  return output_limb;

}

double ForceTransmission::getContactLimbDistance(mc_control::fsm::Controller & ctl_,const std::string & frame,const biRobotTeleop::Limbs limb) const
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);

  const sva::PTransformd & X_0_frame = ctl_.robot().frame(frame).position();
  const auto p_frame = X_0_frame.translation();
  const auto & h = ctl.getHumanPose(ctl.getHumanIndx());


  sch::S_Sphere sphere(0.01);
  sphere.setPosition(p_frame.x(),p_frame.y(),p_frame.z());
  
  auto cvx = h.getConvex(limb);
  
  sch::CD_Pair pair_limb_frame(&cvx,&sphere);

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  return (p1 - p2).norm();

}

Eigen::Vector3d ForceTransmission::getContactPose(mc_control::fsm::Controller & ctl_,
                                                       const biRobotTeleop::Limbs limb_robot,
                                                       const biRobotTeleop::Limbs limb_human)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  auto & robot = ctl.robots().robot();

  const auto h = ctl.getHumanPose(ctl.getDistantHumanIndx());

  auto human_cvx = h.getConvex(limb_human);
  auto robot_cvx = robot.convex(robotPose_.getConvexName(limb_robot));

  sch::CD_Pair pair_limb_frame(&human_cvx,robot_cvx.second.get());

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  Eigen::Vector3d out;
  out << p2[0],p2[1],p2[2];
  return out;

}

void ForceTransmission::resetEnergyState()
{
  e_obs_ = Eigen::Vector6d::Zero();
  e_r_out_ = Eigen::Vector6d::Zero();
  e_s_out_ = Eigen::Vector6d::Zero();
  e_r_in_ = Eigen::Vector6d::Zero();
  e_s_in_ = Eigen::Vector6d::Zero();
  e_d_ = Eigen::Vector6d::Zero(); //dissipated energy
  r_d_ = Eigen::Vector6d::Zero(); //dissipating load

  E_r_out_ = {Eigen::Vector6d::Zero()};
  E_s_out_ = {Eigen::Vector6d::Zero()};
  E_r_in_ = {Eigen::Vector6d::Zero()};
  E_s_in_ = {Eigen::Vector6d::Zero()};
  received_E_r_in_ = {Eigen::Vector6d::Zero()};
  received_E_s_in_ = {Eigen::Vector6d::Zero()};
  E_d_ = {Eigen::Vector6d::Zero()};

  received_force_.clear();
  received_e_r_in_.clear();
  received_e_s_in_.clear();
}

void ForceTransmission::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

  logger.addLogEntry(name() + "_Eobs", [this] () -> const Eigen::Vector6d &  {return e_obs_;});
  logger.addLogEntry(name() + "_Rd", [this] () -> const Eigen::Vector6d &  {return r_d_;});
  logger.addLogEntry(name() + "_Vel" , [this] () -> const sva::MotionVecd & {return vel_;} );

}

void ForceTransmission::addGUI(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();
  gui.addElement({name()},mc_rtc::gui::Checkbox("Active", [this]() -> bool {return !stop_;},[this]() {stop_ = !stop_;}));
  gui.addElement({name()},mc_rtc::gui::NumberInput("Integration Window duration", [this]() -> double {return int_window_;},[this](const double d) {int_window_ = d;}));
  gui.addElement({name()},mc_rtc::gui::Checkbox("Use load", [this]() -> bool {return use_load_;},[this]() {use_load_ = !use_load_;}));
  gui.addElement({name()},mc_rtc::gui::Button("End",[this]() {done_ = true;}));

  for (size_t i = 0 ; i < 6 ; i++ )
  {
  
    gui.addElement(
        {name(), "Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
        mc_rtc::gui::Button("Plot Load axis : " + std::to_string(i),
                            [this,&gui,i]() {
                              gui.addPlot(
                                  name() + "Load axis : " + std::to_string(i),
                                  mc_rtc::gui::plot::X(
                                      "t", [this]() { return static_cast<double>(count) * dt_; }),
                                  mc_rtc::gui::plot::Y(
                                      "R", [this,i]() { return r_d_(i); }, mc_rtc::gui::Color::Red));
                            }),
        mc_rtc::gui::Button("Stop Plot load axis : "  + std::to_string(i) , [this,&gui,&i]() { gui.removePlot(name() + "Load axis : " + std::to_string(i)); })
        
        );
      gui.addElement(
          {name(), "Plots"}, mc_rtc::gui::ElementsStacking::Horizontal,
          mc_rtc::gui::Button("Plot E obs axis : " + std::to_string(i),
                              [this,&gui,i]() {
                                gui.addPlot(
                                    name() + "E obs axis : " + std::to_string(i),
                                    mc_rtc::gui::plot::X(
                                        "t", [this]() { return static_cast<double>(count) * dt_; }),
                                    mc_rtc::gui::plot::Y(
                                        "E obs", [this,i]() { return e_obs_(i); }, mc_rtc::gui::Color::Red));
                              }),
          mc_rtc::gui::Button("Stop Plot E obs axis : "  + std::to_string(i) , [this,&gui,i]() { gui.removePlot(name() + "E obs axis : " + std::to_string(i)); })
          
          );
    
  }

}

void ForceTransmission::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  ctl_.solver().removeTask(task_);

  auto & gui = *ctl_.gui();
  auto & logger = ctl_.logger();

  gui.removeCategory({name()});
  for (size_t i = 0 ; i < 6 ; i++ )
  {
    gui.removePlot(name() + "E obs axis : " + std::to_string(i));
    gui.removePlot(name() + "Load axis : " + std::to_string(i));
  }
  ctl_.datastore().remove(name() +"::MeasuredForce");
  ctl_.datastore().remove(name() +"::E_R_in");
  ctl_.datastore().remove(name() +"::E_S_in");

  logger.removeLogEntry(name() + "_Eobs");
  logger.removeLogEntry(name() + "_Rd");
  logger.removeLogEntry(name() + "_Vel");
}

EXPORT_SINGLE_STATE("ForceTransmission", ForceTransmission)
