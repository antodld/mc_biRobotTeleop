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
  config("activation_threshold")("force",force_activation_threshold_);
  config("activation_threshold")("distance",distance_activation_threshold_);
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

  const mc_rbdyn::Robot & robot = ctl_.robots().robot(robot_indx);
  dt_ = ctl.timeStep;


  addGUI(ctl_);
  addLog(ctl_);

  ctl.getGUIBuilder().addElement({"BiRobotTeleop","ForceTransmission"},mc_rtc::gui::ArrayLabel("e_r_in",[this]() -> const Eigen::VectorXd {return e_r_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("e_s_in",[this]() -> const Eigen::VectorXd {return e_s_in_;} ),
                                                              mc_rtc::gui::ArrayLabel("Measured Force",[this]() -> const Eigen::VectorXd {return measured_force_.vector();} ),
                                                              mc_rtc::gui::ArrayLabel("Target Force",[this]() -> const Eigen::VectorXd {return task_->targetWrench().vector();} )
                                                              );

  ctl.hp_rec_.subsbscribe("e_r_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_r_in");
  ctl.hp_rec_.subsbscribe("e_s_in",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"e_s_in");
  ctl.hp_rec_.subsbscribe("Measured Force",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"Measured Force");
  ctl.hp_rec_.subsbscribe("Measured Force",mc_rtc::gui::Elements::ArrayLabel,{"BiRobotTeleop","ForceTransmission"},"Target Force");

}

bool ForceTransmission::run(mc_control::fsm::Controller & ctl_)
{
  output("OK");

  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  const auto & robotPose = ctl.getRobotPose(ctl.getHumanIndx());
  
  if(!active_) //if not active, look if the force sensors measure a contact
  {
    int filter_indx = 0;
    for (auto & l :force_sensor_limbs_)
    {
      auto limb = biRobotTeleop::str2Limb(l);
      auto frame_name = robotPose.getName(limb);
      const auto w = ctl_.robot().frame(frame_name).wrench();
      activation_force_measurements_[filter_indx].update(w);

      std::pair<double,biRobotTeleop::Limbs> d_pair = {1e9,biRobotTeleop::Limbs::Head};
      for(int i = 1 ; i <= biRobotTeleop::Limbs::RightArm ; i++)
      {
        const auto limb_r = static_cast<biRobotTeleop::Limbs>(i);
        const double d = getContactDistance(ctl_,limb_r,limb).norm();
        if(std::min(d,d_pair.first) < d_pair.first){d_pair = {d,limb_r};}
      }

      if(activation_force_measurements_[filter_indx].eval().vector().norm() > force_activation_threshold_ || 
         d_pair.first < distance_activation_threshold_ ) 
      {
        robot_limb_ = limb;
        //Once a force sensor is in contact, we set the limb in contact and activate the force task;
        if( d_pair.first < distance_activation_threshold_)
        { 
          human_limb_ = limb;
          limb = d_pair.second;
          frame_name = robotPose.getName(limb);
        }
        else
        {
          active_force_measurement_ = &activation_force_measurements_[filter_indx];
          human_limb_ = getContactLimb(ctl,robot_limb_) ;
        } 
        
        mc_rtc::log::info("[{}] adding task",name(),frame_name);

        const std::string distant_robot_frame = ctl.getRobotPose(ctl.getDistantHumanIndx()).getName(human_limb_);
        
        robot_custom_force_sensor_name_ = ctl.robot().name() + "_" + contact_limb_;
        if(!ctl.robot().bodyHasForceSensor(frame_name))
        {
          mc_rbdyn::ForceSensor sensor = mc_rbdyn::ForceSensor(robot_custom_force_sensor_name_,frame_name,sva::PTransformd::Identity());
          ctl.robot().addForceSensor(sensor);
        }

        task_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot().frame(frame_name));
        task_->load(ctl.solver(),config_("task")(ctl.robot().name()));
        task_->velFilterGain(0);

        // const auto distant_robot_name = ctl.getDistantRobotName();
        // task_distant_robot_= std::make_shared<mc_tasks::force::AdmittanceTask>(ctl.robot(distant_robot_name).frame(distant_robot_frame));
        // task_distant_robot_->load(ctl.solver(),config_("task")(distant_robot_name));
        // task_distant_robot_->velFilterGain(0);

        ctl.solver().addTask(task_);
        // ctl.solver().addTask(task_distant_robot_);
        active_ = true;
        break;
      }
      filter_indx +=1;
    }

    return true;
    
  }

  const auto & h = ctl.getHumanPose(ctl.getHumanIndx());

  const sva::PTransformd X_0_frame = task_->frame().position();
  const sva::MotionVecd prev_vel = vel_;

  const auto X_0_h_limb = h.getOffset(human_limb_) * h.getPose(human_limb_);

  const sva::PTransformd X_hContact_frame = X_0_frame * X_0_h_limb.inv();
  const sva::PTransformd X_hContact_rContact = robotPose.getOffset(robot_limb_) * X_hContact_frame ;


  //expressed in unified robot frame
  vel_ = robotPose.getOffset(robot_limb_) * task_->frame().velocity();

  //expressed in human_frame
  auto vel_h_frame = X_0_h_limb * X_0_frame.inv() * task_->frame().velocity();
  

  const double d = getContactDistance(ctl_,robot_limb_,human_limb_).norm(); 
  bool force_deactivate = false;
  if(active_force_measurement_ != nullptr)
  {
    active_force_measurement_->update(task_->frame().wrench());
    force_deactivate = active_force_measurement_->eval().vector().norm() < force_activation_threshold_;
  }


  if(force_deactivate || d > deactivation_threshold_)
  {
    mc_rtc::log::info("[{}] Contact has been broken deactivate force control",name());
    ctl.solver().removeTask(task_);
    // ctl.solver().removeTask(task_distant_robot_);
    active_force_measurement_->reset(sva::ForceVecd::Zero());
    active_force_measurement_ = nullptr;
    resetEnergyState();
    active_ = false;
    return true;
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
  
    const auto R_fb_limb = sva::PTransformd(X_0_h_limb.rotation()) 
                             * sva::PTransformd(X_0_fb.rotation()).inv();
    const auto wrench_fb = ctl.getCalibratedExtWrench(ctl.realRobot());

    measured_force_frame = (X_hContact_frame.inv()
                         * R_fb_limb
                         ).dualMul( sva::ForceVecd(Eigen::Vector3d::Zero(),wrench_fb.force()) );
    
    const auto fs_indx = ctl.robot().data()->forceSensorsIndex.at(robot_custom_force_sensor_name_);
    ctl.robot().data()->forceSensors[fs_indx].wrench(measured_force_frame);
  
  }
  Eigen::VectorXd distant_robot_task_target_force;
  ctl.hp_rec_.getSubscribedData<Eigen::VectorXd>(distant_robot_task_target_force,"Target Force");
  // task_distant_robot_->targetWrench(sva::ForceVecd(distant_robot_task_target_force));

  measured_force_ = robotPose.getOffset(robot_limb_).dualMul(measured_force_frame);

  auto measured_force_h_frame = (X_0_h_limb * X_0_frame.inv()).dualMul(measured_force_frame);

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
  
                                                
  //We target minus the measured force on the distant robot, locally, the distant robot coincide with the local human pose
  //target force is expressed in robot unified frame
  const sva::ForceVecd target_force_h_frame = -getReceivedData<sva::ForceVecd>(received_force_);
  const sva::ForceVecd target_force = X_hContact_rContact.dualMul(target_force_h_frame );


  //Energy is locally expressed in robot unified frame
  updateEnergyState(e_r_out_, -target_force,vel_);
  updateEnergyWindow(e_r_out_,E_r_out_);
  
  updateEnergyState(e_s_out_,-measured_force_,vel_);
  updateEnergyWindow(e_s_out_,E_s_out_);
  
  updateEnergyState(e_r_in_,target_force_h_frame,vel_h_frame);
  updateEnergyWindow(e_r_in_,E_r_in_);
  
  updateEnergyState(e_s_in_,measured_force_h_frame,vel_h_frame);
  updateEnergyWindow(e_s_in_,E_s_in_);


  e_d_ += r_d_.cwiseProduct( prev_vel.vector().cwiseProduct(prev_vel.vector())) * dt_; //P_d = R * v^2
  updateEnergyWindow(e_d_,E_d_);


  e_obs_ =    getReceivedData<Eigen::Vector6d>(received_e_r_in_) 
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
  task_->targetWrench( robotPose.getOffset(robot_limb_).transMul( (target_force + sva::ForceVecd(r_d_.cwiseProduct(vel_.vector())))) );
  task_->targetPose(X_0_frame);
  count++;

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

const biRobotTeleop::Limbs ForceTransmission::getContactLimb(mc_control::fsm::Controller & ctl_,const biRobotTeleop::Limbs & robot_limb) const
{

  biRobotTeleop::Limbs output_limb = biRobotTeleop::Head;
  double min_d = 1e9;

  for (int int_limb = 1 ; int_limb <= biRobotTeleop::Limbs::RightArm ; int_limb++)
  {

    const auto limb = static_cast<biRobotTeleop::Limbs>(int_limb);
    if(limb == biRobotTeleop::Limbs::Pelvis || limb == biRobotTeleop::Limbs::Head){continue;}
  
    auto d = getContactDistance(ctl_,robot_limb,limb).norm();

    if(d < min_d)
    {
      output_limb = limb;
      min_d = d;
    }
  }
  return output_limb;

}


Eigen::Vector3d ForceTransmission::getContactDistance(mc_control::fsm::Controller & ctl_,
                                                       const biRobotTeleop::Limbs limb_robot,
                                                       const biRobotTeleop::Limbs limb_human) const
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  auto & robot = ctl.robots().robot();

  const auto h = ctl.getHumanPose(ctl.getHumanIndx());
  const auto r = ctl.getRobotPose(ctl.getHumanIndx());

  auto human_cvx = h.getConvex(limb_human);
  auto robot_cvx = robot.convex(r.getConvexName(limb_robot));

  sch::CD_Pair pair_limb_frame(&human_cvx,robot_cvx.second.get());

  sch::Point3 p1, p2;
  pair_limb_frame.getClosestPoints(p1,p2);

  Eigen::Vector3d out;
  out << p1[0] - p2[0], p1[1] - p2[1],p1[2] - p2[2];
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
