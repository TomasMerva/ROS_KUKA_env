#include "pushobject.h"

using namespace gazebo;


PushObject::PushObject()
  :  mt(rd()),
    x_cord(650, 900),
    y_cord(-140, 140)
{
  nh = new ros::NodeHandle;

  // service for reseting
  service_reset = nh->advertiseService("object_server", &PushObject::ObjectReset, this);

  // publisher for object's velocity and pose
  object_cord_pub = nh->advertise<kuka_push::object_pose_vel>("object_cord", 1);

  //set fixed orientation and Z coordinate of the block
  new_pose.Rot().X() = 0;
  new_pose.Rot().Y() = 0;
  new_pose.Rot().Z() = 0;
  new_pose.Rot().W() = 1;
  new_pose.Pos().Z() = 0.42;

  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "object";
}

void PushObject::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  this->model = _parent;
  world = this->model->GetWorld();
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PushObject::OnUpdate, this));
}

void PushObject::OnUpdate()
{
    //reading actual pose of the object
    object_pose = this->model->WorldPose();

    object_msg.pose.position.x = object_pose.Pos().X();
    object_msg.pose.position.y = object_pose.Pos().Y();
    object_msg.pose.position.z = object_pose.Pos().Z();

    object_msg.pose.orientation.x = object_pose.Rot().X();
    object_msg.pose.orientation.y = object_pose.Rot().Y();
    object_msg.pose.orientation.z = object_pose.Rot().Z();
    object_msg.pose.orientation.w = object_pose.Rot().W();


    object_linear_vel = this->model->RelativeLinearVel();
    object_msg.twist.linear.x = object_linear_vel.X();
    object_msg.twist.linear.y = object_linear_vel.Y();
    object_msg.twist.linear.z = object_linear_vel.Z();

    object_angular_vel = this->model->RelativeAngularVel();
    object_msg.twist.angular.x = object_angular_vel.X();
    object_msg.twist.angular.y = object_angular_vel.Y();
    object_msg.twist.angular.z = object_angular_vel.Z();

    //publishing the pose and velocity of the block
    object_cord_pub.publish(object_msg);

    // publish tf coordinates for RViz
    static tf2_ros::TransformBroadcaster br;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = object_msg.pose.position.x;
    transformStamped.transform.translation.y = object_msg.pose.position.y;
    transformStamped.transform.translation.z = object_msg.pose.position.z;

    transformStamped.transform.rotation.x = object_msg.pose.orientation.x;
    transformStamped.transform.rotation.y = object_msg.pose.orientation.y;
    transformStamped.transform.rotation.z = object_msg.pose.orientation.z;
    transformStamped.transform.rotation.w = object_msg.pose.orientation.w;

    br.sendTransform(transformStamped);
}


bool PushObject::ObjectReset(kuka_push::object_reset::Request &req,
                             kuka_push::object_reset::Response &resp)
{
  //Generating random numbers for X and Y, Z will remain the same
  new_pose.Pos().X() = x_cord(mt)/1000.0;
  new_pose.Pos().Y() = y_cord(mt)/1000.0;
  //setting the new random position for object in Gazebo
  this->model->SetWorldPose(new_pose);
  resp.confirm = true;
  return true;
}
