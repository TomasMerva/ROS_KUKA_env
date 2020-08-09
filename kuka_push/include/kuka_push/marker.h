#ifndef MARKER_H
#define MARKER_H

#include <kuka_push/include.h>

// Class for generating marker (goal position) in RViz
class RvizMarker
{
  public:
    RvizMarker(ros::NodeHandle* nh);
    void AddMarker(); // generate new position of the marker (goal position)

    visualization_msgs::Marker marker;

  private:
    // Publisher for RViz a Gazebo
    ros::Publisher marker_pub;

    // Generator of random variables
    std::random_device rd;
    std::mt19937 mt;
    std::uniform_int_distribution<int> x_cord;
    std::uniform_int_distribution<int> y_cord;
};
#endif // MARKER_H
