#ifndef KUKAPUSH_ENV_LIB_H
#define KUKAPUSH_ENV_LIB_H

#include <kuka_push/include.h>
#include <kuka_push/marker.h>

//Class for creating KUKA_Push environment.
class kukapush_env
{
  public:
    /* The constructor sets fixed orientation of the gripper, initializes MoveIt
    *  and creates the trajectory publisher and the service client
    *  for reseting object in Gazebo
    */
    kukapush_env(ros::NodeHandle *nh);

    /* The method for receiving an action from the client
    *  and sending a current state of the environment to the client
    */
    bool env_step(kuka_push::rl_env::Request &req,
                  kuka_push::rl_env::Response &resp);

    /* The method for resetting the environment and sending
    *  a current state of the environment thereafter to the client
    */
    void env_reset(kuka_push::rl_env::Response &resp);

    // The callback method for getting the pose and velocity of the object.
    void objectCallback(const kuka_push::object_pose_vel::ConstPtr &msg);

  private:
    void ExecuteAction(geometry_msgs::Pose &action_pose); //Execute an action
    void RewardFunction(); // Binary reward function
    void GetObservation(); // Read and compute poses and velocities
    void SetObservation(kuka_push::rl_env::Response &resp); // prepare a state for sending to client

    //class for generating goal
    RvizMarker marker;

    //Trajectory publisher for JointTrajectoryController
    ros::Publisher pub_joint_publisher;

    //Gazebo
    ros::ServiceClient gazebo_client; // Gazebo client for generating new initial position of the object
    kuka_push::object_reset object_service; // service message for reseting pose of the object

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

    // State of the environment
    geometry_msgs::PoseStamped gripper_pose; // current pose of the robot
    geometry_msgs::Pose action_pose; // desired pose of the robot
    geometry_msgs::Pose object_pose; // current pose of the object
    geometry_msgs::Point goal_position; // goal position
    geometry_msgs::Point object_gripper; // object position relative to gripper
    geometry_msgs::Point object_goal; // object position relative to goal

    geometry_msgs::Twist object_vel; // object linear and angular velocity

    geometry_msgs::Point gripper_prev; // previous gripper's position
    geometry_msgs::Point gripper_speed; // linear velocity of the gripper

    double tolerance = 0.01; // tolerance for computing RewardFunction
    bool found_approach_ik;  // flag for successfully finding IK solution
    double time_move = 0.5;  // desired trajectory time between two points

    /* variables needed for transforming agent's actions <-1.0, 1.0>
    * into physical gripper's coordinates
    */
    double delta_x = 0.19;
    double delta_y = 0.34;
    double delta_z = 0.15;
    double half_range_x = 0.6;
    double half_range_y = -0.34;
    double half_range_z = 0.24;

    // reinforcement learning variables
    float reward = -1.0;
    bool done = false;
};

#endif // KUKAPUSH_ENV_LIB_H
