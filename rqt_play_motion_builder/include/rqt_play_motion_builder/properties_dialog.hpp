/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef RQT_PLAY_MOTION_BUILDER__PROPERTIES_DIALOG_HPP_
#define RQT_PLAY_MOTION_BUILDER__PROPERTIES_DIALOG_HPP_

#include <ui_properties_dialog.h>

#include <QDialog>

#include "play_motion_builder_msgs/srv/store_motion.hpp"

#include "rclcpp/node.hpp"

namespace rqt_play_motion_builder
{
using StoreMotion = play_motion_builder_msgs::srv::StoreMotion;

class MotionProperties : public QDialog
{
  Q_OBJECT

public:
  MotionProperties(QWidget * parent = 0);

  void init(const rclcpp::Node::SharedPtr node);

  void reset();
  void loadYamlName(const std::string & yaml_name);
  void loadMeta(
    const std::string & name,
    const std::string & usage,
    const std::string & description);

private:
  rclcpp::Node::SharedPtr node_;

  QLabel * out_label_;
  Ui::MotionProperties ui_;
  rclcpp::Client<StoreMotion>::SharedPtr store_client_;

private slots:
  void onCancel();
  void onTextChanged(const QString & text);
  void onSaveYAML();

signals:
  void motionStored(QString motion_name);
};
}  // namespace rqt_play_motion_builder

#endif  // RQT_PLAY_MOTION_BUILDER__PROPERTIES_DIALOG_HPP_
