/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef PROPERTIES_DIALOG_PMB_H
#define PROPERTIES_DIALOG_PMB_H

#include <ui_properties_dialog.h>

#include <QDialog>

#include <ros/ros.h>
#include <play_motion_builder_msgs/StoreMotion.h>

namespace pal
{
class MotionProperties : public QDialog
{
  Q_OBJECT
public:
  MotionProperties(QWidget *parent = 0);

  void init(ros::NodeHandle &nh);

  void reset();
  void loadYamlName(const std::string &yaml_name);
  void loadMeta(const std::string &name, const std::string &usage,
                const std::string &description);

private:
  QLabel *out_label_;
  Ui::MotionProperties ui_;
  ros::ServiceClient store_client_;

private slots:
  void onCancel();
  void onTextChanged(const QString &text);
  void onSaveYAML();

signals:
  void motionStored(QString motion_name);
};
}

#endif /*PROPERTIES_DIALOG_PMB_H */
