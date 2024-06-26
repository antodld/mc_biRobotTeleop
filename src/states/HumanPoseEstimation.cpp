#include "HumanPoseEstimation.h"
#include <mc_joystick_plugin/joystick_inputs.h>
#include <mc_rbdyn/RobotLoader.h>
#include "../BiRobotTeleoperation.h"
#include <mc_rtc/gui/Robot.h>

void HumanPoseEstimation::configure(const mc_rtc::Configuration & config)
{

  config_.load(config);
  config_("stiffness",stiffness_);

  if(config.has("target_limbs"))
  {
    std::vector<std::string> limbs;
    target_limbs_.clear();
    config("target_limbs",limbs);
    for (auto & l : limbs)
    {
        target_limbs_.push_back(biRobotTeleop::str2Limb(l));
    }
  }

    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftArm,"L_ARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightArm,"R_ARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightForearm,"R_FOREARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftForearm,"L_FOREARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightHand,"R_HAND_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftHand,"L_HAND_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Pelvis,"TORSO_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Head,"HEAD_LINK");



}

void HumanPoseEstimation::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
    dt_ = ctl.timeStep;

    if(config_.has("human_indx"))
    {
        config_("human_indx",human_indx_);
    }
    else
    {
        human_indx_ = ctl.getHumanIndx();
    }

    auto robots =  ctl.external_robots_;
    humanRobot_name_ = "human_" + std::to_string(human_indx_) + "_estimated";

    mc_rbdyn::RobotModulePtr robot_ptr = mc_rbdyn::RobotLoader::get_robot_module("simple_human",humanRobot_name_);

    mc_rbdyn::Robot & human = robots.get()->load(humanRobot_name_,*robot_ptr);
    human.forwardKinematics();
    human.forwardVelocity();
    human.forwardAcceleration();
    ctl.getGUIBuilder().addElement({"BiRobotTeleop","Estimated Human"},
                            mc_rtc::gui::Robot(humanRobot_name_,
                                [this,&ctl]() -> mc_rbdyn::Robot & {return ctl.external_robots_.get()->robot(humanRobot_name_);})
                            );
    
    ctl.gui()->addElement({"States",name()},mc_rtc::gui::Robot("estimated robot",[this,&ctl]()-> const mc_rbdyn::Robot & {return ctl.external_robots_.get()->robot(humanRobot_name_); }));

    
 
    if( ctl.useRos() )
    {
        mc_rtc::ROSBridge::init_robot_publisher("human_estimation/human_" + std::to_string(human_indx_),dt_,human);
    }
        
    addLog(ctl_);
    h_estimated_ = biRobotTeleop::HumanPose(name());
    h_measured_ = biRobotTeleop::HumanPose(name() + "_measured");
    h_measured_thread_ = biRobotTeleop::HumanPose(name() + "_measured_thread");
    
    estimation_thread_ = 
    std::thread([this,&ctl_]()
        {
            while(run_)
            {
                runThread(ctl_);
                std::this_thread::sleep_for(std::chrono::milliseconds( static_cast<int>(dt_ * 1e3)));
            }

        }
        );

}

bool HumanPoseEstimation::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation&>(ctl_);
    {
        std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
        ctl.updateHumanPose(ctl.getHumanPose(human_indx_),h_measured_);
        h_measured_.setOffset(ctl.getHumanPose(human_indx_).getOffset());
        ctl.updateHumanPose(h_estimated_,ctl.getHumanPose(human_indx_,true));
    }

    output("True");
    return false;
}

void HumanPoseEstimation::runThread(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation&>(ctl_);
    auto ext_robots =  ctl.external_robots_;
    mc_rbdyn::Robot & human = ext_robots.get()->robot(humanRobot_name_);

    {
        std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
        ctl.updateHumanPose(h_measured_,h_measured_thread_);
        h_measured_thread_.setOffset(h_measured_.getOffset());
    }

    const biRobotTeleop::HumanPose & h = h_measured_thread_;

    std::vector<double> dot_q_vec;
    for (auto & j : human.alpha())
    {
        for(auto & qd : j )
        {
            dot_q_vec.push_back(qd);
        }
    }
    dot_q_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dot_q_vec.data(), dot_q_vec.size());
    task_mat_.clear();
    task_vec_.clear();
    task_weight_.clear();
    for(auto & limb : target_limbs_)
    {
        if(!h.limbActive(limb) || !h.limbActive(biRobotTeleop::Limbs::Pelvis)){return;}
        const sva::PTransformd offset = h.getOffset(limb);
        addTransformTask(human,humanRobot_links_.getName(limb), 
                        offset * h.getPose(limb),
                        offset * h.getVel(limb),stiffness_);

    }
    const sva::PTransformd offset = h.getOffset(biRobotTeleop::Limbs::Pelvis);
    addTransformTask(human,"TORSO_LINK",
                    offset * h.getPose(biRobotTeleop::Limbs::Pelvis), 
                    offset * h.getVel(biRobotTeleop::Limbs::Pelvis),stiffness_,2);
    addPostureTask(human,1,1e-1);
    addMinAccTask(human,1e-1);

    const Eigen::VectorXd qdd = solve();

    size_t count = 0;
    for (size_t i = 0 ; i < human.mbc().alphaD.size() ; i++)
    {
        for (size_t j = 0 ; j < human.mbc().alphaD[i].size(); j++)
        {
            human.mbc().alphaD[i][j] = qdd(count);
            count+=1;
        }
    }

    human.eulerIntegration(dt_);
    human.forwardKinematics();
    human.forwardVelocity();
    human.forwardAcceleration();

    {
        std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
        for(int i = 0 ; i <= biRobotTeleop::Limbs::RightArm ; i++ )
        {
            const auto limb = static_cast<biRobotTeleop::Limbs>(i);
            const auto link = humanRobot_links_.getName(limb);
            set_estimated_values(human,link,limb);
            h_estimated_.setOffset(h.getOffset());
        }
    }
    
    if( ctl.useRos() )
    {
        mc_rtc::ROSBridge::update_robot_publisher("human_estimation/human_" + std::to_string(human_indx_),dt_,human);
    }
    
    
}
void HumanPoseEstimation::addLog(mc_control::fsm::Controller & ctl_)
{
    ctl_.logger().addLogEntry("perf_" + name() + "_solver",[this]()-> const double {return solving_perf_;} );
}

void HumanPoseEstimation::addGUI(mc_control::fsm::Controller & ctl_)
{
  
}

void HumanPoseEstimation::teardown(mc_control::fsm::Controller & ctl_)
{
    run_ = false;
    estimation_thread_.join();

    auto & ctl = static_cast<BiRobotTeleoperation&>(ctl_);
    auto robots =  ctl.external_robots_;
    const int indx = robots.get()->robotIndex(humanRobot_name_);
    robots.get()->removeRobot(indx);

}

void HumanPoseEstimation::addTransformTask(mc_rbdyn::Robot & human,const std::string & human_link, const sva::PTransformd & X_0_target, const sva::MotionVecd & targetVel,
                    const double stiffness, const double weight)
{
    rbd::Jacobian jac = rbd::Jacobian(human.mb(),human_link);
    const double damping = 2 * sqrt(stiffness);
    auto J_local = jac.jacobian(human.mb(),human.mbc());
    auto dotJ_local = jac.jacobianDot(human.mb(),human.mbc());
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,human.mb().nrDof());
    Eigen::MatrixXd Jdot = Eigen::MatrixXd::Zero(6,human.mb().nrDof());
    jac.fullJacobian(human.mb(),J_local,J);
    jac.fullJacobian(human.mb(),dotJ_local,Jdot);
    Eigen::Vector6d accRef;
    accRef.segment(3,3) = - stiffness * (human.frame(human_link).position().translation() - X_0_target.translation()) 
                          - damping * (human.bodyVelW(human_link) - targetVel).linear() ;
    accRef.segment(0,3) = - stiffness * sva::rotationError<double>(X_0_target.rotation(),human.frame(human_link).position().rotation())
                          - damping * (human.bodyVelW(human_link) - targetVel).angular() ;


    task_weight_.push_back(weight);
    task_mat_.push_back(J);
    task_vec_.push_back(accRef - Jdot * dot_q_);
}

void HumanPoseEstimation::addPostureTask(mc_rbdyn::Robot & human,const double stiffness, const double weight)
{
    const int n = dot_q_.size() - 6;
    const double damping = 2 * sqrt(stiffness);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n,n+6);
    J.block(0,6,n,n) = Eigen::MatrixXd::Identity(n,n);

    std::vector<double> q_vec;
    size_t count = 0;
    for (auto & j : human.q())
    {
        if(count > 0)
        {
            for(auto & q : j )
            {
                q_vec.push_back(q);
            }
        }
        count+=1;
    }

    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());

    Eigen::VectorXd accRef = -stiffness * q - damping * dot_q_.segment(6,n);
    task_mat_.push_back(J);
    task_vec_.push_back(accRef);
    task_weight_.push_back(weight);
}

void HumanPoseEstimation::addMinAccTask(mc_rbdyn::Robot & human,const double weight)
{
    const int ndof = human.mb().nrDof();
    const Eigen::MatrixXd J = Eigen::MatrixXd::Identity(ndof,ndof);
    const Eigen::VectorXd accR = Eigen::VectorXd::Zero(ndof);
    task_mat_.push_back(J);
    task_vec_.push_back(accR);
    task_weight_.push_back(weight);
}

Eigen::VectorXd HumanPoseEstimation::solve()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (task_mat_.size() == 0)
    {
        mc_rtc::log::critical("[{}, solver] No tasks provided",name());
    }
    const int nvar = task_mat_[0].cols();
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(nvar);

    Eigen::MatrixXd Ginv = Eigen::MatrixXd::Zero(nvar,nvar);
    
    for (size_t i = 0 ; i < task_mat_.size() ; i++)
    {
        Ginv += task_mat_[i].transpose() * task_mat_[i] * task_weight_[i];

    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(Ginv);


    auto rank = lu_decomp.rank();
    if(rank != Ginv.cols())
    {
        mc_rtc::log::critical("[{}] Ginv non invertible",name());
        return Eigen::VectorXd::Zero(qdd.size());
    }
    const Eigen::MatrixXd G = Ginv.inverse();
    for (size_t i = 0 ; i < task_mat_.size() ; i++)
    {
        qdd += task_weight_[i] * G * task_mat_[i].transpose() * task_vec_[i];
    }
    for(int i = 0 ; i < qdd.size() ; i++)
    {
        if (qdd(i) != qdd(i))
        {
            mc_rtc::log::warning("[{}] Estimation failed",name());
            return Eigen::VectorXd::Zero(qdd.size());
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    solving_perf_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    return qdd;
}

EXPORT_SINGLE_STATE("HumanPoseEstimation", HumanPoseEstimation)
