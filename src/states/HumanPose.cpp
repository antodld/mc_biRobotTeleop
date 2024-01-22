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
        }

        config_("robot_device",robot_device_);
        config_("robot_link",robot_link_);
        config_("link_sensor_transfo",X_link_sensor_);
        
    }
    else
    {
        human_sim_rec_.init("human", "human" ,"tcp://" + ip_ + ":" + std::to_string(pub_port_),
                                              "tcp://" + ip_ + ":" + std::to_string(sub_port_));
        human_sim_rec_.setSimulatedDelay(0);
    }
    if(config_.has("human_indx"))
    {
        config_("human_indx",human_indx_);
    }
    robot_name_ = human_indx_ == 1 ? "robot_1" : "robot_2";
    mc_rtc::log::info("[{}] human is human_{}",name(),human_indx_ + 1);

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
        ctl.datastore().get<std::function<sva::PTransformd(std::string)>>(
            "OpenVRPlugin::getPoseByName");

        auto & tracker_vel_func =
        ctl.datastore().get<std::function<sva::MotionVecd(std::string)>>(
            "OpenVRPlugin::getVelocityByName");

        const sva::PTransformd X_0_robotTracker = tracker_pose_func(robot_device_);

        const auto X_0_RobotLink = robot.bodyPosW(robot_link_);


        for (auto & device: human_deviceTolimbs_)
        {
            const sva::PTransformd X_0_tracker =  tracker_pose_func(device.first) ;
            const auto X_robotTracker_tracker = X_0_tracker * X_0_robotTracker.inv();
            const sva::MotionVecd v_tracker = sva::PTransformd( h.getOffset(device.second).rotation() ) * tracker_vel_func(device.first);
            const biRobotTeleop::Limbs limb = device.second;
            h.setPose(limb,X_robotTracker_tracker * X_link_sensor_ * X_0_RobotLink);
            h.setVel(limb,v_tracker);
            
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
