#include "ForceDisplay.h"
#include <mc_joystick_plugin/joystick_inputs.h>

#include "../BiRobotTeleoperation.h"

void ForceDisplay::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config_("receiver_name",receiver_name_);
  config_("delay",t_delay_);
  config("integration_window",int_window_);
  if(config_.has("frame_offset"))
  {
    config_("frame_offset",X_frameR_frame_);
  }
}

void ForceDisplay::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  std::string robot_name = ctl_.robots().robot(0).name();
  if(config_.has("robot"))
  {
    config_("robot",robot_name);
  }
  dt_ = ctl_.timeStep;
  const int robot_indx = ctl_.robots().robotIndex(robot_name);
  
  const mc_rbdyn::Robot & robot = ctl_.robots().robot(robot_indx);
  std::string frame = config_("task")("frame");
  dt_ = ctl.timeStep;
  task_ = std::make_shared<mc_tasks::force::AdmittanceTask>(robot.frame(frame));
  task_->load(ctl.solver(),config_("task"));
  ctl_.solver().addTask(task_);
  ctl_.datastore().make<sva::ForceVecd>(name() +"::MeasuredForce",sva::ForceVecd::Zero());
  ctl_.datastore().make<Eigen::Vector6d>(name() +"::E_R_in",Eigen::Vector6d::Zero());
  ctl_.datastore().make<Eigen::Vector6d>(name() +"::E_S_in",Eigen::Vector6d::Zero());

  addGUI(ctl_);
  addLog(ctl_);

  task_->velFilterGain(0);
  // task_->UseFeedForwardAcceleration(false);

}

bool ForceDisplay::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
  // std::cout << "frame name " << task_->frame().name() << std::endl;
  // mc_rtc::log::info("[{}] vel lin\n{}",name(),vel.linear());
  // mc_rtc::log::info("[{}] vel ang\n{}",name(),vel.angular());

  const sva::PTransformd X_0_frame = task_->frame().position();
  const sva::MotionVecd prev_vel = vel_;
  vel_ = sva::PTransformd(X_0_frame.rotation(),Eigen::Vector3d::Zero()) * task_->frame().velocity();

  const sva::ForceVecd measured_force = task_->measuredWrench();

  if (ctl.datastore().has(receiver_name_ + "::MeasuredForce"))
  {
    received_force_.push_back(ctl.datastore().get<sva::ForceVecd>(receiver_name_ + "::MeasuredForce"));
    if(received_force_.size() > max_buffer_size_)
    {
      received_force_.erase(received_force_.begin());
    }
  }
  if (ctl.datastore().has(receiver_name_ + "::E_R_in"))
  {
    received_e_r_in_.push_back(ctl.datastore().get<Eigen::Vector6d>(receiver_name_ + "::E_R_in"));
    if(received_e_r_in_.size() > max_buffer_size_)
    {
      received_e_r_in_.erase(received_e_r_in_.begin());
    }
  }
  if (ctl.datastore().has(receiver_name_ + "::E_S_in"))
  {
    received_e_s_in_.push_back(ctl.datastore().get<Eigen::Vector6d>(receiver_name_ + "::E_S_in"));
    if(received_e_s_in_.size() > max_buffer_size_)
    {
      received_e_s_in_.erase(received_e_s_in_.begin());
    }
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


  ctl.datastore().assign<Eigen::Vector6d>(name() + "::E_S_in",E_s_in_.back() - E_s_in_.front());
  ctl.datastore().assign<Eigen::Vector6d>(name() + "::E_R_in",E_r_in_.back() - E_r_in_.front());
  ctl.datastore().assign<sva::ForceVecd>(name() + "::MeasuredForce",measured_force);
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

  bool joystick_online = false;

  if(ctl_.datastore().has("Joystick::connected"))
  {
    joystick_online = ctl_.datastore().get<bool>("Joystick::connected");
  }

  if(joystick_online)
  {
    auto & button_func = ctl.datastore().get<std::function<bool(joystickButtonInputs)>>("Joystick::Button");
    done_ =  button_func(joystickButtonInputs::B);
    if(button_func(joystickButtonInputs::A) && static_cast<double>(wait_) * dt_ > 1)
    {
      stop_ = !stop_;
      wait_ = 0;
    }
    wait_++;
    wait_ = wait_%1000;
  }

  output("OK");
  return done_;
}

void ForceDisplay::updateEnergyState(Eigen::Vector6d & energy,const sva::ForceVecd & force, const sva::MotionVecd vel)
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
void ForceDisplay::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & logger = ctl_.logger();

  logger.addLogEntry(name() + "_Eobs", [this] () -> const Eigen::Vector6d &  {return e_obs_;});
  logger.addLogEntry(name() + "_Rd", [this] () -> const Eigen::Vector6d &  {return r_d_;});
  logger.addLogEntry(name() + "_Vel" , [this] () -> const sva::MotionVecd & {return vel_;} );

}

void ForceDisplay::addGUI(mc_control::fsm::Controller & ctl_)
{
  auto & gui = *ctl_.gui();
  gui.addElement({name()},mc_rtc::gui::Checkbox("Active", [this]() -> bool {return !stop_;},[this]() {stop_ = !stop_;}));
  gui.addElement({name()},mc_rtc::gui::NumberInput("delay", [this]() -> double {return t_delay_;},[this](const double d) {t_delay_ = d;}));
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

void ForceDisplay::teardown(mc_control::fsm::Controller & ctl_)
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

EXPORT_SINGLE_STATE("ForceDisplay", ForceDisplay)
