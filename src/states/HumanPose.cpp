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
        config_("human_devices",human_devices_);
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
}

bool HumanPose::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<BiRobotTeleoperation&>(ctl_);

    biRobotTeleop::HumanPose & h = ctl.getHumanPose(ctl.getHumanIndx());

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

        //0t means world frame of tracker
        const sva::PTransformd X_0t_robotTracker = tracker_pose_func(robot_device_);

        const sva::PTransformd X_0t_0 = (ctl.robot().bodyPosW(robot_link_).inv() * X_link_sensor_.inv() * X_0t_robotTracker);


        for (auto & device: human_devices_)
        {
            const sva::PTransformd X_0_tracker = (X_0t_0 * tracker_pose_func(device).inv()).inv() ;
            const sva::MotionVecd v_tracker = sva::PTransformd(X_0t_0.rotation()) * tracker_vel_func(device);
            const biRobotTeleop::Limbs limb = biRobotTeleop::str2Limb(device);
            h.setPose(limb,X_0_tracker);
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
