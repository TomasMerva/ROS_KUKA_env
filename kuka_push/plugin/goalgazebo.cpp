#include "goalgazebo.h"

using namespace gazebo;


// Gazebo plugin for showing a goal position
GoalGazebo::GoalGazebo()
{
  nh = new ros::NodeHandle;
  // subcriber for getting position of the marker (goal)
  sub = nh->subscribe("visualization_marker", 1, &GoalGazebo::Goal, this);

  // Fixed rotation of the marker (does not matter)
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

// Setting a position of the object based on the position of the marker
void GoalGazebo::Goal(const visualization_msgs::Marker& msg)
{
  goal_pose.Pos().X() = msg.pose.position.x;
  goal_pose.Pos().Y() = msg.pose.position.y;
  goal_pose.Pos().Z() = msg.pose.position.z;
  this->model->SetWorldPose(goal_pose);
}
