#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rtc/gui/Input.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tasks/TransformTask.h>

struct ForceDisplay : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void addGUI(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

  template<typename T>
  T getReceivedData(const std::vector<T> & data)
  {
    const int n_delay = static_cast<int>(t_delay_ / dt_);
    const int indx = data.size() - 1 - n_delay;
    if(data.size() <= indx )
    {
      return T::Zero();
    }
    return data[indx];
  }
  /**
   * @brief update energy if the power is positive
   * 
   * @param energy energy to be updated
   * @param force 
   * @param vel 
   */
  void updateEnergyState(Eigen::Vector6d & energy,const sva::ForceVecd & force, const sva::MotionVecd vel);

  void updateEnergyWindow(Eigen::Vector6d & e, std::vector<Eigen::Vector6d> & E)
  {
    E.push_back(e);
    const int n = static_cast<int>(int_window_ / dt_);
    if(E.size() > n)
    {
      E.erase(E.begin());
    }
  }

  double dt_ = 5e-3;
  std::string receiver_name_ = "";
  Eigen::Vector6d e_obs_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d e_r_out_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d e_s_out_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d e_r_in_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d e_s_in_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d e_d_ = Eigen::Vector6d::Zero(); //dissipated energy

  Eigen::Vector6d r_d_ = Eigen::Vector6d::Zero(); //dissipating load
  bool use_load_ = true;
  double t_delay_ = 0;
  std::vector<sva::ForceVecd> received_force_;
  std::vector<Eigen::Vector6d> received_e_r_in_;
  std::vector<Eigen::Vector6d> received_e_s_in_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> task_;

  sva::MotionVecd vel_ = sva::MotionVecd::Zero();

  Eigen::VectorXd weight_ = Eigen::VectorXd::Ones(6);

  sva::PTransformd X_frameR_frame_ = sva::PTransformd::Identity(); //Transform from the receiver control frame to the control frame

  int max_buffer_size_ = 200; 

  mc_rtc::Configuration config_;

  size_t count = 0;
  size_t wait_ = 0;

  double int_window_ = 1;
  std::vector<Eigen::Vector6d> E_r_out_ = {Eigen::Vector6d::Zero()};
  std::vector<Eigen::Vector6d> E_s_out_ = {Eigen::Vector6d::Zero()};
  std::vector<Eigen::Vector6d> E_r_in_ = {Eigen::Vector6d::Zero()};
  std::vector<Eigen::Vector6d> E_s_in_ = {Eigen::Vector6d::Zero()};
  std::vector<Eigen::Vector6d> E_d_ = {Eigen::Vector6d::Zero()};

  bool done_ = false;

  bool stop_ = false;

};
