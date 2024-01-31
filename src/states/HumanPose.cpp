#include "HumanPose.h"
#include <mc_joystick_plugin/joystick_inputs.h>
#include <mc_rbdyn/RobotLoader.h>
#include "../BiRobotTeleoperation.h"
#include <mc_rtc/gui/Robot.h>

void HumanPose::configure(const mc_rtc::Configuration & config)
{

    config_.load(config);

}

void HumanPose::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation &>(ctl_);
    const auto & config = ctl.getGlobalConfig();

    human_indx_ = ctl.getHumanIndx();
    if(config_.has("human_indx"))
    {
        config_("human_indx",human_indx_);
    }
    robot_name_ = human_indx_ == 1 ? "robot_1" : "robot_2";
    mc_rtc::log::info("[{}] human is human_{}",name(),human_indx_ + 1);

    offline_threshold_ = config("offline_threshold",100);

    if(config.has("human_sim"))
    {
        config("human_sim")("ip",ip_);
        config("human_sim")("pub_port",pub_port_);
        config("human_sim")("sub_port",sub_port_);
        config("human_sim")("active",human_sim_);
    }

    if(!human_sim_)
    {
        std::vector<std::vector<std::string>> map;
        config_("human_devices",map);
        for (auto & limb : map )
        {
            human_deviceTolimbs_[limb[0]] = biRobotTeleop::str2Limb(limb[1]);
            mc_rtc::log::info("[{}] limb {} mapped to device {}",name(),limb[1],limb[0]);
            online_data_count_[human_deviceTolimbs_[limb[0]]] = 0;
        }

        config_("robot_device",robot_device_);
        config_("robot_link",robot_link_);
        config_("link_sensor_transfo")(robot_name_,X_link_sensor_);
        
    }
    else
    {
        human_sim_rec_.init("human", "human" ,"tcp://" + ip_ + ":" + std::to_string(pub_port_),
                                              "tcp://" + ip_ + ":" + std::to_string(sub_port_));
        human_sim_rec_.setSimulatedDelay(0);
    }


    auto & gui = *ctl.gui();
    gui.addElement(this, {"States",name(), "Robot sensor offset"},

                    mc_rtc::gui::ArrayInput(
                        "translation [m]", {"x", "y", "z"},
                        [this]() -> const Eigen::Vector3d & { return X_link_sensor_.translation(); },
                        [this](const Eigen::Vector3d & t) { X_link_sensor_.translation() = t; }),
                    mc_rtc::gui::ArrayInput(
                        "rotation [deg]", {"r", "p", "y"},
                        [this]() -> Eigen::Vector3d {
                        return mc_rbdyn::rpyFromMat(X_link_sensor_.rotation()) * 180. / mc_rtc::constants::PI;
                        },
                        [this](const Eigen::Vector3d & rpy) {
                        X_link_sensor_.rotation() = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.);
                        }));
    
    gui.addElement({"States",name(), "Robot sensor offset"},mc_rtc::gui::Transform("Expected robot sensor pose",[this,&ctl_]() -> sva::PTransformd
                                                                                    {
                                                                                        mc_rbdyn::Robot & robot = ctl_.robots().robot(robot_name_);
                                                                                        const auto X_0_RobotLink = robot.bodyPosW(robot_link_);
                                                                                        return X_link_sensor_ * X_0_RobotLink;
                                                                                    }));


}

bool HumanPose::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation&>(ctl_);


    biRobotTeleop::HumanPose & h = ctl.getHumanPose(human_indx_);
    mc_rbdyn::Robot & robot = ctl.robots().robot(robot_name_);

    if(ctl.robots().hasRobot("human_1"))
    {   
        output("True");
        return true;
    }

    if(!human_sim_)
    {

        auto & tracker_pose_func =
        ctl.datastore().get<std::function<sva::PTransformd(const std::string &)>>(
            "OpenVRPlugin::getPoseByName");

        auto & tracker_vel_func =
        ctl.datastore().get<std::function<sva::MotionVecd(const std::string &)>>(
            "OpenVRPlugin::getVelocityByName");

        auto & has_tracker_func =
        ctl.datastore().get<std::function<bool(const std::string &)>>(
            "OpenVRPlugin::deviceHasName");

        auto & tracker_online_func =
        ctl.datastore().get<std::function<bool(const std::string &)>>(
            "OpenVRPlugin::deviceOnline");

                
        const sva::PTransformd X_0_robotTracker = (has_tracker_func(robot_device_) && tracker_online_func(robot_device_)) ? 
                                                    tracker_pose_func(robot_device_) : sva::PTransformd::Identity();

        const auto X_0_RobotLink = robot.bodyPosW(robot_link_);

        for (auto & device: human_deviceTolimbs_)
        {
            if(has_tracker_func(device.first) && tracker_online_func(device.first))
            {
                const sva::PTransformd X_0_trackerRaw =  tracker_pose_func(device.first) ;
                const auto X_robotTracker_tracker = X_0_trackerRaw * X_0_robotTracker.inv();
                const sva::PTransformd X_0_tracker = X_robotTracker_tracker * X_link_sensor_ * X_0_RobotLink;
                const sva::MotionVecd v_tracker = sva::PTransformd( X_0_tracker.rotation() ).inv() * tracker_vel_func(device.first);
                const biRobotTeleop::Limbs limb = device.second;

                if( checkNorm(X_0_robotTracker) || checkNorm(X_0_trackerRaw) || checkNorm(X_0_tracker) )
                {
                    // mc_rtc::log::warning("[{}] tracker on limb {} not received\nKeeping the previous pose\n{}",name(),biRobotTeleop::limb2Str(limb),h.getPose(limb));
                    // mc_rtc::log::info(checkNorm(h.getPose(limb)));
                    h.setVel(limb,sva::MotionVecd::Zero());
                    if(online_data_count_[limb] >= offline_threshold_ && h.limbActive(limb))
                    {
                        mc_rtc::log::warning("[{}] tracker on limb {} not received",name(),biRobotTeleop::limb2Str(limb));
                        h.setLimbActiveState(limb,false);
                    }
                    else if (online_data_count_[limb] < offline_threshold_)
                    {
                        online_data_count_[limb] += 1;
                    }
                }
                else
                {
                    h.setPose(limb,X_0_tracker);
                    h.setVel(limb,v_tracker);
                    online_data_count_[limb] == 0;
                    h.setLimbActiveState(limb,true);
                }
            }
            
        }
    }
    else if(human_sim_ && human_sim_rec_.getRobot().name() == "human")
    {
        const mc_rbdyn::Robot & human_robot = human_sim_rec_.getRobot();
        // mc_rtc::log::info("here");
        ctl.updateHumanPose(human_robot,h);
    }
   
    output("True");
    return false;
 
}
void HumanPose::addLog(mc_control::fsm::Controller & ctl_)
{

}

void HumanPose::addGUI(mc_control::fsm::Controller & ctl_)
{
  
}

void HumanPose::teardown(mc_control::fsm::Controller & ctl_)
{

}



EXPORT_SINGLE_STATE("HumanPose", HumanPose)
