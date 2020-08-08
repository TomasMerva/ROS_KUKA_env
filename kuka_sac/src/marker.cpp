#include "kuka_sac/marker.h"

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_int_distribution<int> x_cord(470, 765);
std::uniform_int_distribution<int> y_cord(-350, 350);
std::uniform_int_distribution<int> z_cord(110, 600);


RvizMarker::RvizMarker(ros::NodeHandle* nh)
{
  marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

  marker.header.frame_id = "/base_link";
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.lifetime = ros::Duration();
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );


}

//adding new marker at a random position
void RvizMarker::AddMarker(){
  marker.header.stamp = ros::Time::now();
  x = x_cord(mt)/1000.0;
  y = y_cord(mt)/1000.0;
  z = z_cord(mt)/1000.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker_pub.publish(marker);
  transform.setOrigin(tf::Vector3(x, y, z));
  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(0), "base_link", "marker"));
//  std::cout << "marker.pose \n" << marker.pose.position << std::endl;
}


