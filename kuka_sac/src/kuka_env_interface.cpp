#include "kuka_sac/kuka_env_interface.h"



kuka_env_interface::kuka_env_interface(ros::NodeHandle* nh)
  : marker(nh),
    KUKA_robot(nh)
{
}

bool kuka_env_interface::env_step(kuka_sac::service::Request &req,
                                  kuka_sac::service::Response &resp)
{
  if(req.reset)
  {
    env_reset(resp);
    timestep = req.timestep;
  }
  else
  {
    ExecuteAction(req);
    timestep = req.timestep;
    ros::Duration(0.03).sleep();
    KUKA_robot.GetObservation(&marker);
    RewardFunction();
    KUKA_robot.SetObservation(resp);
  }

  return true;
}

void kuka_env_interface::env_reset(kuka_sac::service::Response &resp)
{
  marker.AddMarker();
  // Moving arm into default position
  KUKA_robot.action_pose.position.x = 0.445426;
  KUKA_robot.action_pose.position.y = 1.65521e-06;
  KUKA_robot.action_pose.position.z = 0.81;
  KUKA_robot.Move(&marker);
  // Waiting so the arm is in default position
  ros::Duration(1.5).sleep();
  KUKA_robot.GetObservation(&marker);
  RewardFunction();
  KUKA_robot.SetObservation(resp);
}

void kuka_env_interface::ExecuteAction(kuka_sac::service::Request &req)
{
  KUKA_robot.action_pose.position.x = (static_cast<double>(req.action[0])*delta_x + delta_x) + 0.47;
  KUKA_robot.action_pose.position.y = (static_cast<double>(req.action[1])*delta_y + delta_y) - 0.35;
  KUKA_robot.action_pose.position.z = (static_cast<double>(req.action[2])*delta_z + delta_z) + 0.11;
  KUKA_robot.Move(&marker);
}

void kuka_env_interface::RewardFunction()
{
  if(((abs(static_cast<float>(KUKA_robot.current_pose.pose.position.x - KUKA_robot.goal_pose.position.x)))<= epsilon) &&
     ((abs(static_cast<float>(KUKA_robot.current_pose.pose.position.y - KUKA_robot.goal_pose.position.y)))<= epsilon) &&
     ((abs(static_cast<float>(KUKA_robot.current_pose.pose.position.z - KUKA_robot.goal_pose.position.z)))<= epsilon)  )
  {
    KUKA_robot.reward = 0.0;
    KUKA_robot.done = true;
  }
  else
  {
    KUKA_robot.reward = -1.0;
    KUKA_robot.done = false;
  }

}
