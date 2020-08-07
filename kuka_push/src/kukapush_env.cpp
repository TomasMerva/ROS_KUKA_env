#include <kuka_push/kukapush_env_lib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kukapush_env");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  kukapush_env env(&nh);
  ros::ServiceServer service = nh.advertiseService("kukapush_server",
                                              &kukapush_env::env_step, &env);


  ros::Subscriber sub = nh.subscribe("object_cord", 1,
                                    &kukapush_env::objectCallback, &env);

  ros::waitForShutdown();
  return 0;
}
