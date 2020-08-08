#ifndef KUKA_ENV_MOVEMENT_H
#define KUKA_ENV_MOVEMENT_H

#include <kuka_sac/include.h>
#include <kuka_sac/marker.h>
#include <kuka_sac/service.h>

class kuka_env_movement
{
  public:
    kuka_env_movement(ros::NodeHandle* nh);
    void Move(RvizMarker* marker);
    void GetObservation(RvizMarker *marker);
    void SetObservation(kuka_sac::service::Response &resp);

    geometry_msgs::Pose goal_pose;
    geometry_msgs::Pose action_pose;
    geometry_msgs::Pose gripper_pose;
    geometry_msgs::Pose object_world_pose;
    geometry_msgs::Point object_gripper_position;
    geometry_msgs::PoseStamped current_pose;

    float reward = -1.0;
    bool done = false;

    bool ik_error;

    moveit::planning_interface::MoveGroupInterface group;

  private:
    //Trajectory publisher for controller
    ros::Publisher pub_joint_publisher;

    //ROS formats
    trajectory_msgs::JointTrajectory arm_command;
    trajectory_msgs::JointTrajectoryPoint desired_configuration;
    std::vector<double> goal_joint_values;

    //Moveit
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model_ptr;
    robot_state::RobotStatePtr current_state_ptr;
    robot_state::RobotStatePtr goal_state_ptr;
    robot_state::JointModelGroup* joint_model_group; //mozno treba pridat const

    bool found_approach_ik;
    double time_move = 0.5;

};

#endif // KUKA_ENV_MOVEMENT_H
