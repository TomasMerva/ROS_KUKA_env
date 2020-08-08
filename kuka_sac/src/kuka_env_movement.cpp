#include "kuka_sac/kuka_env_movement.h"


kuka_env_movement::kuka_env_movement(ros::NodeHandle* nh)
  : group("manipulator"),
    robot_model_loader("robot_description"),
    current_state_ptr(group.getCurrentState()),
    goal_state_ptr(group.getCurrentState())
{
  pub_joint_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("position_trajectory_controller/command",1);

  robot_model_ptr = robot_model_loader.getModel();
  joint_model_group = robot_model_ptr->getJointModelGroup(group.getName());

  desired_configuration.positions.resize(NUM_OF_JOINTS);
  arm_command.joint_names.resize(NUM_OF_JOINTS);

  action_pose.orientation.x = 0.706846;
  action_pose.orientation.y = 0.000587617;
  action_pose.orientation.z = 0.707367;
  action_pose.orientation.w = 0.000508517;
}

void kuka_env_movement::Move(RvizMarker* marker)
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

    /* Setting message for controller
     */
    arm_command.header.stamp = ros::Time::now();
    arm_command.header.frame_id = "base_link";
    arm_command.points.resize(1);
    arm_command.points[0] = desired_configuration;
    arm_command.points[0].time_from_start = ros::Duration(time_move);

    // Publish trajectory into controller
    pub_joint_publisher.publish(arm_command);
  }//if found_approach_ik

  else
  {
    ROS_WARN("No IK found");
    //std::cout << "marker_pose: \n" << marker->marker.pose.position;
    std::cout << "actions are: \n" << action_pose.position << std::endl;
    ik_error = true;

  }//else found_approach_ik
}

void kuka_env_movement::GetObservation(RvizMarker *marker)
{
  current_pose = group.getCurrentPose();

  //Setting gripper_pose
  gripper_pose.position.x = current_pose.pose.position.x;
  gripper_pose.position.y = current_pose.pose.position.y;
  gripper_pose.position.z = current_pose.pose.position.z;
  gripper_pose.orientation.x = current_pose.pose.orientation.x;
  gripper_pose.orientation.y = current_pose.pose.orientation.y;
  gripper_pose.orientation.z = current_pose.pose.orientation.z;
  gripper_pose.orientation.w = current_pose.pose.orientation.w;

  //Setting object_world_pose
  object_world_pose.position.x = marker->marker.pose.position.x;
  object_world_pose.position.y = marker->marker.pose.position.y;
  object_world_pose.position.z = marker->marker.pose.position.z;
  object_world_pose.orientation.x = marker->marker.pose.orientation.x;
  object_world_pose.orientation.y = marker->marker.pose.orientation.y;
  object_world_pose.orientation.z = marker->marker.pose.orientation.z;
  object_world_pose.orientation.w = marker->marker.pose.orientation.w;

  //Setting object_gripper_position
  object_gripper_position.x = marker->marker.pose.position.x - current_pose.pose.position.x;
  object_gripper_position.y = marker->marker.pose.position.y - current_pose.pose.position.y;
  object_gripper_position.z = marker->marker.pose.position.z - current_pose.pose.position.z;

  //Setting goal position
  goal_pose.position.x = marker->marker.pose.position.x;
  goal_pose.position.y = marker->marker.pose.position.y;
  goal_pose.position.z = marker->marker.pose.position.z;
}

void kuka_env_movement::SetObservation(kuka_sac::service::Response &resp)
{
  //Assign gripper_pose with respect to world frame to Service request
  resp.state[0] = static_cast<float>(gripper_pose.position.x);
  resp.state[1] = static_cast<float>(gripper_pose.position.y);
  resp.state[2] = static_cast<float>(gripper_pose.position.z);
  resp.state[3] = static_cast<float>(gripper_pose.orientation.x);
  resp.state[4] = static_cast<float>(gripper_pose.orientation.y);
  resp.state[5] = static_cast<float>(gripper_pose.orientation.z);
  resp.state[6] = static_cast<float>(gripper_pose.orientation.w);

//  //Assign object_pose with respect to world frame to Service respuest
//  resp.state[7] = static_cast<float>(object_world_pose.position.x);
//  resp.state[8] = static_cast<float>(object_world_pose.position.y);
//  resp.state[9] = static_cast<float>(object_world_pose.position.z);
//  resp.state[10] = static_cast<float>(object_world_pose.orientation.x);
//  resp.state[11] = static_cast<float>(object_world_pose.orientation.y);
//  resp.state[12] = static_cast<float>(object_world_pose.orientation.z);
//  resp.state[13] = static_cast<float>(object_world_pose.orientation.w);

  //Assign object_position with respect to gripper to Service request
  resp.state[7] = static_cast<float>(object_gripper_position.x);
  resp.state[8] = static_cast<float>(object_gripper_position.y);
  resp.state[9] = static_cast<float>(object_gripper_position.z);

  //Time
  //resp.state[17] = static_cast<float>(time_move);

  //Asign desired_goal
  resp.desired_goal[0] = static_cast<float>(goal_pose.position.x);
  resp.desired_goal[1] = static_cast<float>(goal_pose.position.y);
  resp.desired_goal[2] = static_cast<float>(goal_pose.position.z);

  //Assign achieved_goal
  resp.achieved_goal[0] = static_cast<float>(gripper_pose.position.x);
  resp.achieved_goal[1] = static_cast<float>(gripper_pose.position.y);
  resp.achieved_goal[2] = static_cast<float>(gripper_pose.position.z);

  resp.reward = reward;
  resp.done = done;
}
