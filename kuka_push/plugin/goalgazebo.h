#ifndef GOAL_PLUGIN_H
#define GOAL_PLUGIN_H

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <ignition/math4/ignition/math/Vector3.hh>
#include <ignition/math4/ignition/math/Pose3.hh>
#include <visualization_msgs/Marker.h>


namespace gazebo
{

class GoalGazebo : public ModelPlugin
{
  public:
    GoalGazebo();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    void OnUpdate(); //reading and broadcasting position of the object

    void Goal(const visualization_msgs::Marker& msg);

  private:
    //Gazebo objects
    physics::ModelPtr model;
    gazebo::physics::WorldPtr world;

    //Ros objects
    ros::NodeHandle *nh;
    ros::Subscriber sub;
    visualization_msgs::Marker marker;

    //variables for poses of the object
    ignition::math::Pose3d goal_pose;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;



};
GZ_REGISTER_MODEL_PLUGIN(GoalGazebo)
}


#endif // GOAL_PLUGIN_H
