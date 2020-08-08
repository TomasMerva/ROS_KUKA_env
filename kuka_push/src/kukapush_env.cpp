#include <kuka_push/kukapush_env_lib.h>

// Initialize KUKA_push environment
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kukapush_env");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Creating service server for interaction between an agent and the environment.
  // The server represents the environment.
  kukapush_env env(&nh);
  ros::ServiceServer service = nh.advertiseService("kukapush_server",
                                              &kukapush_env::env_step, &env);

  // Creating the Subscriber for getting the pose of the block from Gazebo
  ros::Subscriber sub = nh.subscribe("object_cord", 1,
                                    &kukapush_env::objectCallback, &env);

  ros::waitForShutdown();
  return 0;
}
