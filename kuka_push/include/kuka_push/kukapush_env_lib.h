#ifndef KUKAPUSH_ENV_LIB_H
#define KUKAPUSH_ENV_LIB_H

#include <kuka_push/include.h>
#include <kuka_push/marker.h>


class kukapush_env
{
  public:
    kukapush_env(ros::NodeHandle *nh);
    bool env_step(kuka_push::rl_env::Request &req,
                  kuka_push::rl_env::Response &resp);
    void env_reset(kuka_push::rl_env::Response &resp);
    void objectCallback(const kuka_push::object_pose_vel::ConstPtr &msg);

  private:
    void ExecuteAction(geometry_msgs::Pose &action_pose);
    void RewardFunction();
    void GetObservation();
    void SetObservation(kuka_push::rl_env::Response &resp);

    //class for generating goal
    RvizMarker marker;

    //Trajectory publisher for controller
    ros::Publisher pub_joint_publisher;


    //Gazebo
    ros::ServiceClient gazebo_client;
    kuka_push::object_reset object_service;

    //Moveit
    moveit::planning_interface::MoveGroupInterface group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr robot_model_ptr;
    robot_state::RobotStatePtr current_state_ptr;
    robot_state::RobotStatePtr goal_state_ptr;
    const robot_state::JointModelGroup* joint_model_group;

    //ROS formats
    trajectory_msgs::JointTrajectory arm_command;
    trajectory_msgs::JointTrajectoryPoint desired_configuration;
    std::vector<double> goal_joint_values;

    //RL State
    geometry_msgs::PoseStamped gripper_pose; //current pose of the robot
    geometry_msgs::Pose action_pose; //desired pose of the robot
    geometry_msgs::Pose object_pose; //current pose of the object
    geometry_msgs::Point goal_position; //goal position
    geometry_msgs::Point object_gripper; //object position relative to gripper
    geometry_msgs::Point object_goal; //object position relative to goal

    geometry_msgs::Twist object_vel; //object linear and angular velocity

    geometry_msgs::Point gripper_prev;
    geometry_msgs::Point gripper_speed;

    double tolerance = 0.05;
    bool found_approach_ik;
    double time_move = 0.5;

    //transforming agent's actions
    double delta_x = 0.19;
    double delta_y = 0.34;
    double delta_z = 0.15;
    double half_range_x = 0.6;
    double half_range_y = -0.34;
    double half_range_z = 0.24;

    //RL variables
    float reward = -1.0;
    bool done = false;    
};

#endif // KUKAPUSH_ENV_LIB_H
