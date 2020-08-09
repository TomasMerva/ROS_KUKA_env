# ROS_KUKA_env

## Overview
This repository contains an implementation of the Soft Actor-Critic and Hindsight Experience Replay algorithms alongside with the RL robotics environments KUKA_Push within ROS. [video](https://youtu.be/GN2U0PE8QBk)

![KUKA_Push environment](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/kukapush.png?raw=true "KUKA_Push environment")

The system 
## Dependency
- **ROS Melodic**
- **ros_control**
- **MoveIt**
- **Trac-IK**
- **TensorFlow (Version 1.14.0)

### Overview of ROS mechanism for KUKA Push
![KUKA_Push mechanism](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/scheme.png?raw=true "KUKA_Push mechanism")

## Result
The learning performance of SAC+HER in the KUKA_Push environment

![KUKA_Push performance](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/kukapush_performance.png?raw=true "KUKA_Push performance")

