#ifndef KUKA_ENV_INTERFACE_H
#define KUKA_ENV_INTERFACE_H

#include <kuka_sac/include.h>
#include <kuka_sac/service.h>
#include <kuka_sac/kuka_env_movement.h>
#include <kuka_sac/marker.h>
#include <controller_manager_msgs/SwitchController.h>

class kuka_env_interface
{
  public:
    kuka_env_interface(ros::NodeHandle* nh);
    bool env_step(kuka_sac::service::Request &req,
                  kuka_sac::service::Response &resp);
    void env_reset(kuka_sac::service::Response &resp);
    void ExecuteAction(kuka_sac::service::Request &req);
    void RewardFunction();


  private:
    //Custom classes
    RvizMarker marker;
    kuka_env_movement KUKA_robot;

    //Server for RL agent

    float epsilon = 0.01;
    int timestep=0;
    int max_timesteps=49;

    double delta_x = 0.1475;
    double delta_y = 0.35;
    double delta_z = 0.245;

};

#endif // KUKA_ENV_INTERFACE_H
