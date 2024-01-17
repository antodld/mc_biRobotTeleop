mc_BiRobotTeleop
=====

This is the controller repo the biRobot teleoperation experiment

Dependencies
----

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [biRobotTeleoperation utils](https://github.com/antodld/biRobotTeleopTask)
- [OpenVRPlugin](https://github.com/antodld/mc_OpenVR_plugin)
- [puppet_robot](https://github.com/antodld/mc_puppet)
- [puppet_robot descritpion](https://github.com/antodld/puppet_description)
- [mc_human](https://github.com/jrl-umi3218/mc_human)

All dependencies can be installed using the [super build extension](https://github.com/antodld/biRobotTeleoperation-superbuild)


Using the controller
---

The global configuration file should include those parameters (which can vary if you are using robot_1 or robot_2)

```yaml

MainRobot:
  name: robot_2 #or robot_1
  module: JVRC1 # desired robot module
Enabled: BiRobotTeleoperation

Timestep: 0.002

GUIServer:
  IPC:
    Socket: /tmp/mc_rtc_2
  TCP:
    # Binding host, * binds to all interfaces
    Host: "*" 
    # Binding ports, the first is used for PUB socket and the second for
    # the PULL socket
    Ports: [4242, 4343]
    Timestep: 0.005


distant_controller:
  ip: localhost #host address of the distant controller
  sub_port : 4242
  pub_port: 4343
  human_name: human_2 #or human_1
local_controller:
  human_name: human_1 #or human_2

human_sim:
  active: true #if true the sensor data are overwritten by an external human controller
  ip: localhost #host address of the simulated human controller
  sub_port : 4242
  pub_port: 4343 
```
