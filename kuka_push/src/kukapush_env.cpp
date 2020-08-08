#include <kuka_push/kukapush_env_lib.h>

/*! \fn int main(int argc, char **argv)
    \brief Initialize KUKA_push environment
    \return The exit status of the program as an integer
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kukapush_env");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  /* Creating service server for interaction between an agent and the environment.
     The server represents the environment.
  */
  kukapush_env env(&nh);
  ros::ServiceServer service = nh.advertiseService("kukapush_server",
                                              &kukapush_env::env_step, &env);

  /* Creating the Subscriber for getting the pose of the block from Gazebo
  */
  ros::Subscriber sub = nh.subscribe("object_cord", 1,
                                    &kukapush_env::objectCallback, &env);

  ros::waitForShutdown();
  return 0;
}
