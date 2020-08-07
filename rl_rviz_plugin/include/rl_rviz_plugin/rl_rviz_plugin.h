#ifndef RlPlugin_H
#define RlPlugin_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <kuka_push/rlplugin_msg.h>

#include <rviz/panel.h>
#include <QLabel>
#include <QGridLayout>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>
#include <QFormLayout>

namespace rl_rviz_plugin
{

class RlPlugin : public rviz::Panel
{
Q_OBJECT
public:
  RlPlugin( QWidget* parent = 0 );

protected:
  QLabel *epoch_label;
  QLabel *episode_label;
  QLabel *prevscore_label;
  QLabel *maxscore_label;

  QLineEdit *epoch_value;
  QLineEdit *episode_value;
  QLineEdit *prevscore_value;
  QLineEdit *maxscore_value;

  QGridLayout *grid;
  QGroupBox *box;

  QFormLayout *layout;

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_epoch;
  void epochCallback(const kuka_push::rlplugin_msg &msg);


public Q_SLOTS:

Q_SIGNALS:

};

}

#endif // RL_RVIZ_PLUGIN_H
