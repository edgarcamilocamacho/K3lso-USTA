# K3lso Quadruped

K3lso is a quadruped robot, powered by Jetson NX.  This is BLDC version.

Slack channel: https://join.slack.com/t/newworkspace-7gn1112/shared_invite/zt-hqn0zss3-pZaL3pkLpwtRk7jjMMkjIw

Visit the main repository: https://github.com/raess1/K3lso-Quadruped

# K3lso-ROS

## Requirements

Tested with:

* Ubuntu 18.04, Linux Mint 19.3
* ROS Melodic

Ros packages:

``` bash
sudo apt-get install ros-melodic-robot-localization ros-melodic-yocs-velocity-smoother
```

## Testing

Walking demo in RVIZ:
``` bash
roslaunch k3lso_config bringup.launch rviz:=true
```

Run the Gazebo environment:
``` bash
roslaunch k3lso_config gazebo.launch 
```

Run the teleop node:
``` bash
roslaunch champ_teleop teleop.launch
```




