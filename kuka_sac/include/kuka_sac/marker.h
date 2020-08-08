#ifndef MARKER_H
#define MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <iostream>
#include <random>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

class RvizMarker
{
  public:
    RvizMarker(ros::NodeHandle* nh);
    void AddMarker();

    visualization_msgs::Marker marker;
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::TransformListener listener;

    tf::StampedTransform gripper_transform;
    tf::StampedTransform object_world_transform;
    tf::StampedTransform object_gripper_transform;


    geometry_msgs::Pose gripper_pose;
    geometry_msgs::Pose object_world_pose;
    geometry_msgs::Point object_gripper_position;

  private:
    ros::Publisher marker_pub;

    float max_x = 0.75;
    float min_x = 0.45;
    float max_y = 0.35;
    float min_y = -0.35;
    float max_z = 0.7;
    float min_z = 0.1;

    double x;
    double y;
    double z;

};

#endif // MARKER_H
