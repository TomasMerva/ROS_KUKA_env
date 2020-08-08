#include "goalgazebo.h"

using namespace gazebo;



GoalGazebo::GoalGazebo()
{
  nh = new ros::NodeHandle;
  sub = nh->subscribe("visualization_marker", 1, &GoalGazebo::Goal, this);

  goal_pose.Rot().X() = 0;
  goal_pose.Rot().Y() = 0;
  goal_pose.Rot().Z() = 0;
  goal_pose.Rot().W() = 1;
}

void GoalGazebo::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  this->model = _parent;
  world = this->model->GetWorld();
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GoalGazebo::OnUpdate, this));
}

void GoalGazebo::OnUpdate()
{


}

void GoalGazebo::Goal(const visualization_msgs::Marker& msg)
{
  goal_pose.Pos().X() = msg.pose.position.x;
  goal_pose.Pos().Y() = msg.pose.position.y;
  goal_pose.Pos().Z() = msg.pose.position.z;
  this->model->SetWorldPose(goal_pose);
}
