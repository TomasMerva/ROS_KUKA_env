# ROS_KUKA_env

## Overview
This repository contains the implementation of the **Soft Actor-Critic** and **Hindsight Experience Replay** algorithms alongside with the RL robotics environment **KUKA_Push** within ROS. The goal of the environment is to push the block towards a goal position using the KUKA end-effector. [video](https://youtu.be/GN2U0PE8QBk)

![KUKA_Push environment](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/kukapush.png?raw=true "KUKA_Push environment")

## Dependency
- **ROS Melodic**
- **ros_control**
- **MoveIt**
- **Trac-IK**
- **TensorFlow (Version 1.14.0)**

### Overview of ROS mechanism for KUKA_Push
The agent and environment are two separate ROS nodes that communicate through ROS Services. The agent fulfils the role of the client, whereas the environment functions as the server. Before each episode, the agent sends a request with reset flag set to 1. As a result, the environment ignores the received actions and prepares itself for new episode. After resetting the environment, the agent chooses actions [X,Y,Z] based on a current state of the environment and sends them in the form of a request to the service server (the environment). Consequently, the desired [X,Y,Z] coordinates are assigned to the Trac-IK solver in order to compute a set of position waypoints that are applied to the robot for the time of 30ms. The new state is recorded after those 30 ms and thereafter sent back with other information to the agent. The interaction loop lasts 40ms.

In order to get a state of the block, the object gazebo plugin has been created. The plugin functions as a publisher and a service server as well. The publisher is responsible for reading and sending object's pose and velocities each 1ms, wheareas the service server responds to the environment's request for resetting the object's position. To change a goal position in Gazebo, the other plugin has been created that subscribes the marker (representing a goal in RViz) topic. For better overview of the learning process, the RViz plugin has been created so an user can check the learning process of the agent.
![KUKA_Push mechanism](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/scheme.png?raw=true "KUKA_Push mechanism")

## Start guide
1. Launch gazebo simulation: `roslaunch kuka_gazebo gazebo_basic.launch`
2. Start MoveIt and RViz: `roslaunch kuka_gazebo gazebo_moveit.launch`
3. Start the environment: `roslaunch kuka_push environment.launch`
4. Start the training process: `roslaunch kuka_push agent_training.launch`
---
5. Check your path for saving and reloading the neural network model
  5.1. `kuka_push/scripts/kuka_agent.py` -> path where you want to save the model
  5.2. `kuka_push/scripts/kuka_trained.py` -> path from where you want to load the model

## Result
The learning performance of SAC+HER in the KUKA_Push environment for 30 epochs (every epoch = 500 episodes = 500*100 timesteps):

![KUKA_Push performance](https://raw.githubusercontent.com/TomasMerva/ROS_KUKA_env/master/img/kukapush_performance.png?raw=true "KUKA_Push performance")

## Acknowledgement
I would like to thank **OpenAI SpinningUp tutorials** and **kuka_experimental** for their tutorials, code and packages.
https://spinningup.openai.com/en/latest/

http://wiki.ros.org/kuka_experimental

