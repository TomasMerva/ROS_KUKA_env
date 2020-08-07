#ifndef MARKER_H
#define MARKER_H

#include <kuka_push/include.h>

class RvizMarker
{
  public:
    RvizMarker(ros::NodeHandle* nh);
    void AddMarker();

    visualization_msgs::Marker marker;

  private:
    ros::Publisher marker_pub;
    ros::Publisher goal_pub;

    geometry_msgs::Point msg;

    double x;
    double y;

    std::random_device rd;
    std::mt19937 mt;
    std::uniform_int_distribution<int> x_cord;
    std::uniform_int_distribution<int> y_cord;
};
#endif // MARKER_H

