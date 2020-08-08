#include "kuka_sac/kuka_env_interface.h"



//bool env_step(kuka_sac::service::Request &req,
//              kuka_sac::service::Response &resp)
//{
//  std::cout << req.action << std::endl;
//  resp.state = 2.0;
//  return true;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kuka_env");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  kuka_env_interface KUKA(&nh);
  ros::ServiceServer service = nh.advertiseService("env_server",
                                                  &kuka_env_interface::env_step, &KUKA);



  ros::waitForShutdown();
  return 0;
}
