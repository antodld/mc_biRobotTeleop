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
  if(config_.has("frame_offset"))
  {
    config_("frame_offset",X_frameR_frame_);
  }
}

void ForceTransmission::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  std::string robot_name = ctl_.robots().robot(0).name();
  
  robotPose_.load(config_(robot_name)("limb_map"));

  dt_ = ctl_.timeStep;

  config_(robot_name)("force_sensor_frames",force_sensor_frames_);
  for (int _ = 0 ; _ < force_sensor_frames_.size(); _++)
  {
    activation_force_measurements_.push_back(mc_filter::LowPass<sva::ForceVecd>(dt_,1));
  }

  const int robot_indx = ctl_.robots().robotIndex(robot_name);
  
  const mc_rbdyn::Robot & robot = ctl_.robots().robot(robot_indx);
  dt_ = ctl.timeStep;


  addGUI(ctl_);
  addLog(ctl_);

  ctl.gui()->addElement({"BiRobotTeleop","ForceTransmission"},mc_rtc::gui::ArrayLabel("e_r_in",[this]() -> const Eigen::VectorXd {return e_r_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("e_s_in",[this]() -> const Eigen::VectorXd {return e_s_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("Measured Force",[this]() -> const Eigen::VectorXd {return measured_force_.vector();} ),
                                                              mc_rtc::gui::Label("Contact Limb",[this]() -> const std::string & {return contact_limb_;} ));

  ctl.hp_rec_.subsbscribe("e_r_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_r_in");
  ctl.hp_rec_.subsbscribe("e_s_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_s_in");
  ctl.hp_rec_.subsbscribe("Measured Force",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"Measured Force");
  ctl.hp_rec_.subsbscribe("Contact Limb",mc_rtc::gui::Elements::Label,{"BiRobotTeleop","ForceTransmission"},"Contact Limb");


}

bool ForceTransmission::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  // std::cout << "frame name " << task_->frame().name() << std::endl;
  // mc_rtc::log::info("[{}] vel lin\n{}",name(),vel.linear());
  // mc_rtc::log::info("[{}] vel ang\n{}",name(),vel.angular());
  
  std::string distant_contact_limb = "None";
  ctl.hp_rec_.getSubscribedData<std::string>(distant_contact_limb,"Contact Limb");

  if(!active_) //if not active, look if the force sensors measure a contact
  {
    int filter_indx = 0;
    for (auto & f :force_sensor_frames_)
    {
      const auto w = ctl_.robot().frame(f).wrench();
      activation_force_measurements_[filter_indx].update(w);
      if(activation_force_measurements_[filter_indx].eval().vector().norm() > activation_threshold_) 
      {
        //Once a force sensor is in contact, we set the limb in contact and activate the force task;
        mc_rtc::log::info("[{}] force measured on frame {}, adding task",name(),f);
        const auto contact_limb = getContactLimb(ctl,f);
        contact_limb_ = biRobotTeleop::limb2Str(contact_limb);
        task_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot().frame(f));
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
      mc_rtc::log::info("[{}] force measured on limb {}, adding corresponding task",name(),distant_contact_limb);
      const std::string frame = robotPose_.getName(biRobotTeleop::str2Limb(distant_contact_limb));   
      task_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot().frame(frame));
      task_->load(ctl.solver(),config_("task"));  
      task_->velFilterGain(0);
      ctl.solver().addTask(task_);
      active_ = true;
    } 
  }

  if(active_)
  {

    const sva::PTransformd X_0_frame = task_->frame().position();
    const sva::MotionVecd prev_vel = vel_;
    vel_ = sva::PTransformd(X_0_frame.rotation(),Eigen::Vector3d::Zero()) * task_->frame().velocity();

    const sva::ForceVecd measured_force = task_->measuredWrench();
    
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

    if(task_->frame().hasForceSensor()){measured_force_ = task_->frame().wrench();}
    else
    {
      //if the link is not equipped with F/T sensing, we use an estimator that will set the global estimatied force in the mbc at the fb
      const auto X_0_fb = ctl.robot().posW();
      const auto X_0_frame = task_->frame().position();
      measured_force_ = (X_0_frame * X_0_fb.inv()).dualMul(ctl.robot().mbc().force[0]);
    
    }

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

    const sva::ForceVecd target_force = X_frameR_frame_.dualMul( getReceivedData<sva::ForceVecd>(received_force_));
    updateEnergyState(e_r_out_,-target_force,vel_);
    updateEnergyWindow(e_r_out_,E_r_out_);
    
    updateEnergyState(e_s_out_,-measured_force,vel_);
    updateEnergyWindow(e_s_out_,E_s_out_);
    
    updateEnergyState(e_r_in_,target_force,vel_);
    updateEnergyWindow(e_r_in_,E_r_in_);
    
    updateEnergyState(e_s_in_,measured_force,vel_);
    updateEnergyWindow(e_s_in_,E_s_in_);


    e_d_ += r_d_.cwiseProduct( prev_vel.vector().cwiseProduct(prev_vel.vector())) * dt_; //P_d = R * v^2
    updateEnergyWindow(e_d_,E_d_);


    e_obs_ = getReceivedData<Eigen::Vector6d>(received_e_r_in_) 
              + getReceivedData<Eigen::Vector6d>(received_e_s_in_) 
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
    task_->targetWrench( target_force + sva::ForceVecd(r_d_.cwiseProduct(vel_.vector())));
    task_->targetPose(X_0_frame);
    count++;
  
  }
  
  output("OK");
  return done_;
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
