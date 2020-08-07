#include "kuka_push/marker.h"


RvizMarker::RvizMarker(ros::NodeHandle* nh)
  : mt(rd()),
    x_cord(700, 850),
    y_cord(-100, 100)
{
  marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);
  goal_pub = nh->advertise<geometry_msgs::Point>("goal_pub", 1);

  marker.header.frame_id = "/world";
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  // Set fixed marker orientation
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set fixed height of the marker
  marker.pose.position.z = 0.42;

  marker.lifetime = ros::Duration();
}

void RvizMarker::AddMarker()
{
  marker.header.stamp = ros::Time::now();
  x = x_cord(mt)/1000.0;
  y = y_cord(mt)/1000.0;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker_pub.publish(marker);

  msg.x = marker.pose.position.x;
  msg.y = marker.pose.position.y;
  msg.z = marker.pose.position.z;
  goal_pub.publish(msg);
}