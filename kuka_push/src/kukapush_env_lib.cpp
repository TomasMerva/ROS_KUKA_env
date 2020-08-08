#include <kuka_push/kukapush_env_lib.h>
#include <chrono>

// Random goal position
std::random_device rd;
std::mt19937 mt(rd());
std::uniform_int_distribution<int> x_cord(600, 980); //range of random x_coordinate
std::uniform_int_distribution<int> y_cord(-340, 340); // range of random y_coordinate

// Constructor
kukapush_env::kukapush_env(ros::NodeHandle *nh)
  : marker(nh),
    group("manipulator"),
    robot_model_loader("robot_description"),
    current_state_ptr(group.getCurrentState()),
    goal_state_ptr(group.getCurrentState())
{
  //trajectory publisher for JointTrajectoryController
  pub_joint_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("position_trajectory_controller/command",1);

  //Gazebo client for reseting the object's pose and generating new goal
  gazebo_client = nh->serviceClient<kuka_push::object_reset>("object_server");

  //Moveit
  robot_model_ptr = robot_model_loader.getModel();
  joint_model_group = robot_model_ptr->getJointModelGroup(group.getName());

  desired_configuration.positions.resize(NUM_OF_JOINTS);
  arm_command.joint_names.resize(NUM_OF_JOINTS);

  //set fixed orientation of gripper
  action_pose.orientation.x = 0.706846;
  action_pose.orientation.y = 0.000587617;
  action_pose.orientation.z = 0.707367;
  action_pose.orientation.w = 0.000508517;
}


bool kukapush_env::env_step(kuka_push::rl_env::Request &req,
                            kuka_push::rl_env::Response &resp)
{
  // check if agent requires resetting the environment or interacting with it
  if(req.reset)
  {
    env_reset(resp); //reset the environment
  }
  else // interact with the environment
  {
    // transforming desired agent's actions into physical coordinates
    action_pose.position.x = (static_cast<double>(req.action[0])*delta_x + delta_x) + half_range_x;
    action_pose.position.y = (static_cast<double>(req.action[1])*delta_y + delta_y) + half_range_y;
    action_pose.position.z = (static_cast<double>(req.action[1])*delta_z + delta_z) + half_range_z;
    if(action_pose.position.z < 0.43)
    {
      action_pose.position.z = 0.43;
    }
    ExecuteAction(action_pose); // execute client's action
    ros::Duration(0.03).sleep(); // wait 30ms for action having an effect
    GetObservation(); // get and compute state
    RewardFunction(); // compute reward
    SetObservation(resp); // prepare state for sending
  }
  return true;
}

// ------------------------------------------------
void kukapush_env::env_reset(kuka_push::rl_env::Response &resp)
{
  //generate new goal
  marker.AddMarker();

  action_pose.position.x = 0.62;
  action_pose.position.y = 0.0;
  action_pose.position.z = 0.43;
  //go to home pose
  ExecuteAction(action_pose);
  ros::Duration(0.8).sleep();

  //generate new initial position of the object
  object_service.request.reset = true;
  gazebo_client.call(object_service);
  //ros::Duration(0.1).sleep(); //just to be sure, probably useless
  gripper_pose = group.getCurrentPose();
  gripper_prev.x = gripper_pose.pose.position.x;
  gripper_prev.y = gripper_pose.pose.position.y;
  gripper_prev.z = gripper_pose.pose.position.z;

  GetObservation(); // get and compute state
  RewardFunction(); // compute reward
  SetObservation(resp); // prepare state for sending
}

// ------------------------------------------------
void kukapush_env::ExecuteAction(geometry_msgs::Pose &action_pose)
{
  //Calculate joint values based on action_pose
  found_approach_ik = goal_state_ptr->setFromIK(joint_model_group, action_pose, 0.04);
  //If the solution has been found, continuou
  if(found_approach_ik)
  {
    goal_state_ptr->copyJointGroupPositions("manipulator", goal_joint_values);

    // Assign joint_values to joint_names
    std::stringstream joint_name;
    for(int i = 0; i< NUM_OF_JOINTS; ++i){
      joint_name.str("");
      joint_name << "joint_a" <<  (i + 1);
      desired_configuration.positions[i] = goal_joint_values[i];
      arm_command.joint_names[i] = joint_name.str();
    }

    // Setting message for controller
    arm_command.header.stamp = ros::Time::now();
    arm_command.header.frame_id = "base_link";
    arm_command.points.resize(1);
    arm_command.points[0] = desired_configuration;
    arm_command.points[0].time_from_start = ros::Duration(time_move);

    // Publish trajectory into controller
    pub_joint_publisher.publish(arm_command);
  }

  else
  {
    ROS_WARN("No IK found");
    std::cout << "actions are: \n" << action_pose.position << std::endl;
  }
}

// ------------------------------------------------
void kukapush_env::RewardFunction()
{
  // Checking if the object is within tolerance at the goal position
  if(((abs(object_pose.position.x - goal_position.x)) <= tolerance) &&
     ((abs(object_pose.position.y - goal_position.y)) <= tolerance) &&
     ((abs(object_pose.position.z - goal_position.z)) <= tolerance))
  {
    reward = 0.0; // positive learning signal
    done = true; // flag that the goal is accomplished
  }
  else
  {
    reward = -1.0; //negative learning signal
    done = false; // flag that the goal is NOT accomplished
  }
}


void kukapush_env::GetObservation()
{
  //get current pose of the gripper
  gripper_pose = group.getCurrentPose();

  //get goal position
  goal_position = marker.marker.pose.position;

  //object position relative to gripper
  object_gripper.x = gripper_pose.pose.position.x - object_pose.position.x;
  object_gripper.y = gripper_pose.pose.position.y - object_pose.position.y;
  object_gripper.z = gripper_pose.pose.position.z - object_pose.position.z;

  gripper_speed.x = ((abs(gripper_pose.pose.position.x - gripper_prev.x))/0.04)/10;
  gripper_speed.y = ((abs(gripper_pose.pose.position.y - gripper_prev.y))/0.04)/10;
  gripper_speed.z = ((abs(gripper_pose.pose.position.z - gripper_prev.z))/0.04)/10;

  gripper_prev.x = gripper_pose.pose.position.x;
  gripper_prev.y = gripper_pose.pose.position.y;
  gripper_prev.z = gripper_pose.pose.position.z;
}

void kukapush_env::SetObservation(kuka_push::rl_env::Response &resp)
{
  //assign gripper's pose
  resp.state[0] = gripper_pose.pose.position.x;
  resp.state[1] = gripper_pose.pose.position.y;
  resp.state[2] = gripper_pose.pose.position.z;
  resp.state[3] = gripper_pose.pose.orientation.x;
  resp.state[4] = gripper_pose.pose.orientation.y;
  resp.state[5] = gripper_pose.pose.orientation.z;
  resp.state[6] = gripper_pose.pose.orientation.w;

  //assign object's pose
  resp.state[7] = object_pose.position.x;
  resp.state[8] = object_pose.position.y;
  resp.state[9] = object_pose.position.z;
  resp.state[10] = object_pose.orientation.x;
  resp.state[11] = object_pose.orientation.y;
  resp.state[12] = object_pose.orientation.z;
  resp.state[13] = object_pose.orientation.w;

  //assign object's position relative to gripper's position
  resp.state[14] = object_gripper.x;
  resp.state[15] = object_gripper.y;
  resp.state[16] = object_gripper.z;

  // assign linear velocities of the object
  resp.state[17] = object_vel.linear.x;
  resp.state[18] = object_vel.linear.y;
  resp.state[19] = object_vel.linear.z;

  // assign angular velocities of the object
  resp.state[20] = object_vel.angular.x;
  resp.state[21] = object_vel.angular.y;
  resp.state[22] = object_vel.angular.z;

  // assign linear velocities of the gripper
  resp.state[23] = gripper_speed.x;
  resp.state[24] = gripper_speed.y;
  resp.state[25] = gripper_speed.z;

  //Asign desired_goal
  resp.desired_goal[0] = goal_position.x;
  resp.desired_goal[1] = goal_position.y;
  resp.desired_goal[2] = goal_position.z;

  //Assign achieved_goal
  resp.achieved_goal[0] = object_pose.position.x;
  resp.achieved_goal[1] = object_pose.position.y;
  resp.achieved_goal[2] = object_pose.position.z;

  resp.reward = reward;
  resp.done = done;
}

void kukapush_env::objectCallback(const kuka_push::object_pose_vel::ConstPtr &msg)
{
    // subscribe object's pose and velocities from Gazebo object's plugins
    object_pose.position = msg->pose.position;
    object_pose.orientation = msg->pose.orientation;

    object_vel.linear = msg->twist.linear;
    object_vel.angular = msg->twist.angular;
}
