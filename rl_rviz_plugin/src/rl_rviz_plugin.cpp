#include "rl_rviz_plugin/rl_rviz_plugin.h"

namespace rl_rviz_plugin
{

RlPlugin::RlPlugin( QWidget* parent)
  : rviz::Panel( parent )
{
  sub_epoch = nh.subscribe("/rl_states", 1, &RlPlugin::epochCallback, this);

  grid = new QGridLayout(this);
  this->setLayout(grid);
  box = new QGroupBox("RL state" ,this);
  grid->addWidget(box, 0, 0);

  layout = new QFormLayout(box);

  epoch_label = new QLabel("Epoch:");
  episode_label = new QLabel("Episode:");
  prevscore_label = new QLabel("Prev. Score:");
  maxscore_label = new QLabel("Max Score:");

  epoch_value = new QLineEdit();
  episode_value = new QLineEdit();
  prevscore_value = new QLineEdit();
  maxscore_value = new QLineEdit();

  epoch_value->setReadOnly(true);
  episode_value->setReadOnly(true);
  prevscore_value->setReadOnly(true);
  maxscore_value->setReadOnly(true);

  layout->addRow(epoch_label, epoch_value);
  layout->addRow(episode_label, episode_value);
  layout->addRow(prevscore_label, prevscore_value);
  layout->addRow(maxscore_label, maxscore_value);

  this->setLayout(layout);
}



void RlPlugin::epochCallback(const kuka_push::rlplugin_msg &msg)
{
  QString epoch_str;
  QString episode_str;
  QString prevscore_str;
  QString maxscore_str;


  epoch_str = QString::number(msg.epoch, 'f', 1);
  epoch_value->setText(epoch_str);

  episode_str = QString::number(msg.episode, 'f', 1);
  episode_value->setText(episode_str);

  prevscore_str = QString::number((double)msg.prev_score, 'g', 1);
  prevscore_value->setText(prevscore_str);

  maxscore_str = QString::number((double)msg.max_score, 'g', 1);
  maxscore_value->setText(maxscore_str);

}



}//namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rl_rviz_plugin::RlPlugin,rviz::Panel)
