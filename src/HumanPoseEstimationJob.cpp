#include "HumanPoseEstimationJob.h"

HumanPoseEstimationJob::HumanPoseEstimationJob(BiRobotTeleoperation & ctl,
                                               const mc_rbdyn::Robots & extRobotsCtl,
                                               const std::string & humanRobotName,
                                               const mc_rtc::Configuration & config,
                                               const std::string & name)
: humanRobot_name_(humanRobotName)
{
  result.init(name);
  dt_ = ctl.timeStep;
  config("stiffness", stiffness_);

  if(config.has("target_limbs"))
  {
    std::vector<std::string> limbs;
    target_limbs_.clear();
    config("target_limbs", limbs);
    for(auto & l : limbs)
    {
      target_limbs_.push_back(biRobotTeleop::str2Limb(l));
      mc_rtc::log::info("Limb {} added as {}", l, biRobotTeleop::str2Limb(l));
    }
  }

  auto estimationModule = ctl.config()("estimation_module", std::string{"simple_human"});

  if(estimationModule == "simple_human")
  {
    // for simple_human
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftArm, "L_ARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightArm, "R_ARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightForearm, "R_FOREARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftForearm, "L_FOREARM_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightHand, "R_HAND_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftHand, "L_HAND_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Pelvis, "TORSO_LINK");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Head, "HEAD_LINK");
  }
  else
  {
    // for human
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftArm, "LArmLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightArm, "RArmLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightForearm, "RForearmLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftForearm, "LForearmLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::RightHand, "RHandLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::LeftHand, "LHandLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Pelvis, "TorsoLink");
    humanRobot_links_.setName(biRobotTeleop::Limbs::Head, "HeadLink");
  }

  if(config.has("human_indx"))
  {
    config("human_indx", human_indx_);
  }
  else
  {
    human_indx_ = ctl.getHumanIndx();
  }

  ext_robots = mc_rbdyn::Robots::make();
  extRobotsCtl.copy(*ext_robots);
  auto & human = ext_robots->robot(humanRobot_name_);
  human.forwardKinematics();
  human.forwardVelocity();
  human.forwardAcceleration();
}

HumanPoseEstimationResult HumanPoseEstimationJob::computeJob()
{
  mc_rbdyn::Robot & human = ext_robots->robot(humanRobot_name_);

  const biRobotTeleop::HumanPose & h = input_.h_measured_;

  std::vector<double> dot_q_vec;
  for(auto & j : human.alpha())
  {
    for(auto & qd : j)
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
    if(!h.limbActive(limb) || !h.limbActive(biRobotTeleop::Limbs::Pelvis))
    {
      // return;
      continue;
    }
    const sva::PTransformd offset = h.getOffset(limb);
    addTransformTask(human, humanRobot_links_.getName(limb), offset * h.getPose(limb), offset * h.getVel(limb),
                     stiffness_);
  }
  const sva::PTransformd offset = h.getOffset(biRobotTeleop::Limbs::Pelvis);
  auto torsoName = humanRobot_links_.getName(biRobotTeleop::Limbs::Pelvis);
  addTransformTask(human, torsoName, offset * h.getPose(biRobotTeleop::Limbs::Pelvis),
                   offset * h.getVel(biRobotTeleop::Limbs::Pelvis), stiffness_, 2);
  addPostureTask(human, 1, 1e-1);
  addMinAccTask(human, 1e-1);

  auto start_solve = mc_rtc::clock::now();
  const Eigen::VectorXd qdd = solve();
  dt_solve_ = mc_rtc::elapsed_ms_count(start_solve);

  size_t count = 0;
  for(size_t i = 0; i < human.mbc().alphaD.size(); i++)
  {
    for(size_t j = 0; j < human.mbc().alphaD[i].size(); j++)
    {
      human.mbc().alphaD[i][j] = qdd(count);
      count += 1;
    }
  }

  human.eulerIntegration(dt_);
  human.forwardKinematics();
  human.forwardVelocity();
  human.forwardAcceleration();

  // put feet on the floor
  auto posW = human.posW();
  auto X_0_LeftFoot = human.frame("LeftSole").position();
  auto X_0_RightFoot = human.frame("RightSole").position();
  auto X_0_FootMid = sva::interpolate(X_0_LeftFoot, X_0_RightFoot, 0.5); // mid point between both feet
  // move floating base down such that the midpoint between feet is on the floor
  posW.translation().z() -= X_0_FootMid.translation().z();
  // human.posW(posW);

  // Set result
  for(int i = 0; i <= biRobotTeleop::Limbs::RightArm; i++)
  {
    const auto limb = static_cast<biRobotTeleop::Limbs>(i);
    const auto link = humanRobot_links_.getName(limb);
    set_estimated_values(human, result.h_estimated_, link, limb);
    result.h_estimated_.setOffset(h.getOffset());
  }
  return result;
}

void HumanPoseEstimationJob::addTransformTask(mc_rbdyn::Robot & human,
                                              const std::string & human_link,
                                              const sva::PTransformd & X_0_target,
                                              const sva::MotionVecd & targetVel,
                                              const double stiffness,
                                              const double weight)
{
  rbd::Jacobian jac = rbd::Jacobian(human.mb(), human_link);
  const double damping = 2 * sqrt(stiffness);
  auto J_local = jac.jacobian(human.mb(), human.mbc());
  auto dotJ_local = jac.jacobianDot(human.mb(), human.mbc());
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, human.mb().nrDof());
  Eigen::MatrixXd Jdot = Eigen::MatrixXd::Zero(6, human.mb().nrDof());
  jac.fullJacobian(human.mb(), J_local, J);
  jac.fullJacobian(human.mb(), dotJ_local, Jdot);
  Eigen::Vector6d accRef;
  accRef.segment(3, 3) = -stiffness * (human.frame(human_link).position().translation() - X_0_target.translation())
                         - damping * (human.bodyVelW(human_link) - targetVel).linear();
  accRef.segment(0, 3) =
      -stiffness * sva::rotationError<double>(X_0_target.rotation(), human.frame(human_link).position().rotation())
      - damping * (human.bodyVelW(human_link) - targetVel).angular();

  task_weight_.push_back(weight);
  task_mat_.push_back(J);
  task_vec_.push_back(accRef - Jdot * dot_q_);
}

void HumanPoseEstimationJob::addPostureTask(mc_rbdyn::Robot & human, const double stiffness, const double weight)
{
  const int n = dot_q_.size() - 6;
  const double damping = 2 * sqrt(stiffness);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, n + 6);
  J.block(0, 6, n, n) = Eigen::MatrixXd::Identity(n, n);

  std::vector<double> q_vec;
  size_t count = 0;
  for(auto & j : human.q())
  {
    if(count > 0)
    {
      for(auto & q : j)
      {
        q_vec.push_back(q);
      }
    }
    count += 1;
  }

  Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());

  Eigen::VectorXd accRef = -stiffness * q - damping * dot_q_.segment(6, n);
  task_mat_.push_back(J);
  task_vec_.push_back(accRef);
  task_weight_.push_back(weight);
}

void HumanPoseEstimationJob::addMinAccTask(mc_rbdyn::Robot & human, const double weight)
{
  const int ndof = human.mb().nrDof();
  const Eigen::MatrixXd J = Eigen::MatrixXd::Identity(ndof, ndof);
  const Eigen::VectorXd accR = Eigen::VectorXd::Zero(ndof);
  task_mat_.push_back(J);
  task_vec_.push_back(accR);
  task_weight_.push_back(weight);
}

Eigen::VectorXd HumanPoseEstimationJob::solve()
{
  if(task_mat_.size() == 0)
  {
    mc_rtc::log::critical("[{}, solver] No tasks provided", name());
  }
  const int nvar = task_mat_[0].cols();
  Eigen::VectorXd qdd = Eigen::VectorXd::Zero(nvar);

  Eigen::MatrixXd Ginv = Eigen::MatrixXd::Zero(nvar, nvar);

  for(size_t i = 0; i < task_mat_.size(); i++)
  {
    Ginv += task_mat_[i].transpose() * task_mat_[i] * task_weight_[i];
  }

  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(Ginv);

  auto rank = lu_decomp.rank();
  if(rank != Ginv.cols())
  {
    mc_rtc::log::critical("[{}] Ginv non invertible", name());
    return Eigen::VectorXd::Zero(qdd.size());
  }
  const Eigen::MatrixXd G = Ginv.inverse();
  for(size_t i = 0; i < task_mat_.size(); i++)
  {
    qdd += task_weight_[i] * G * task_mat_[i].transpose() * task_vec_[i];
  }
  for(int i = 0; i < qdd.size(); i++)
  {
    if(qdd(i) != qdd(i))
    {
      mc_rtc::log::warning("[{}] Estimation failed", name());
      return Eigen::VectorXd::Zero(qdd.size());
    }
  }
  return qdd;
}

void HumanPoseEstimationJob::set_estimated_values(mc_rbdyn::Robot & human,
                                                  biRobotTeleop::HumanPose & h_estimated_,
                                                  const std::string & link,
                                                  biRobotTeleop::Limbs limb)
{
  const sva::PTransformd & offset = h_estimated_.getOffset(limb);

  h_estimated_.setPose(limb, offset.inv() * human.frame(link).position());
  h_estimated_.setVel(limb, sva::MotionVecd::Zero());
  h_estimated_.setAcc(limb, sva::MotionVecd::Zero());
  // h_estimated_.setVel(limb, human.bodyVelW(link));
  // const sva::MotionVecd v = h_estimated_.getVel(limb);
  // const sva::MotionVecd & acc_body = human.bodyAccB(link);
  // h_estimated_.setAcc(limb,
  //                      sva::PTransformd(human.frame(link).position().inv().rotation()) * acc_body +
  //                      sva::MotionVecd(Eigen::Vector3d::Zero(),v.angular().cross(v.linear())));
}
