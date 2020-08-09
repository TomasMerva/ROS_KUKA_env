#ifndef PUSHOBJECT_H
#define PUSHOBJECT_H

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ignition/math4/ignition/math/Vector3.hh>
#include <ignition/math4/ignition/math/Pose3.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kuka_push/object_reset.h>
#include <kuka_push/object_pose_vel.h>

namespace gazebo
{

// Gazebo plugin for the block: getting its position and setting new random position of it
class PushObject : public ModelPlugin
{
  public:
    PushObject();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    void OnUpdate();

    //service function for setting random position of the block
    bool ObjectReset(kuka_push::object_reset::Request &req,
                     kuka_push::object_reset::Response &resp);


  private:
    //Gazebo objects
    physics::ModelPtr model;
    gazebo::physics::WorldPtr world;

    //Ros objects
    ros::NodeHandle *nh;
    ros::ServiceServer service_reset;
    ros::Publisher object_cord_pub;

    //Custom msg
    kuka_push::object_pose_vel object_msg;

    geometry_msgs::Pose object_pose_msg;
    geometry_msgs::Twist object_vel_msg;
    geometry_msgs::TransformStamped transformStamped;

    //variables for poses of the object
    ignition::math::Pose3d new_pose;
    ignition::math::Pose3d object_pose;
    ignition::math::Vector3d object_linear_vel;
    ignition::math::Vector3d object_angular_vel;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Generator of random variables
    std::random_device rd;
    std::mt19937 mt;
    std::uniform_int_distribution<int> x_cord;
    std::uniform_int_distribution<int> y_cord;

};
GZ_REGISTER_MODEL_PLUGIN(PushObject)
}


#endif // PUSHOBJECT_H
