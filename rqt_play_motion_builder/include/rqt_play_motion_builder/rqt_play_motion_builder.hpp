#ifndef RQT_PLAY_MOTION_BUILDER__RQT_PLAY_MOTION_BUILDER_HPP_
#define RQT_PLAY_MOTION_BUILDER__RQT_PLAY_MOTION_BUILDER_HPP_

#include <rqt_gui_cpp/plugin.h>
#include <ui_rqt_play_motion_builder.h>

#include <QMenu>

#include "play_motion_builder_msgs/action/build_motion.hpp"
#include "play_motion_builder_msgs/action/run_motion.hpp"
#include "play_motion_builder_msgs/srv/change_joints.hpp"
#include "play_motion_builder_msgs/srv/edit_motion.hpp"
#include "play_motion_builder_msgs/srv/list_joint_groups.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"
#include "play_motion2_msgs/srv/list_motions.hpp"
#include "play_motion2_msgs/msg/motion.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp/client.hpp"

#include "rqt_play_motion_builder/properties_dialog.hpp"

namespace rqt_play_motion_builder
{
class RQTPlayMotionBuilder : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RQTPlayMotionBuilder();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings &, qt_gui_cpp::Settings &) const override
  {
  }
  virtual void restoreSettings(const qt_gui_cpp::Settings &, const qt_gui_cpp::Settings &) override
  {
  }

private:
  using BuildMotion = play_motion_builder_msgs::action::BuildMotion;
  using RunMotion = play_motion_builder_msgs::action::RunMotion;

  using ListJointGroups = play_motion_builder_msgs::srv::ListJointGroups;
  using EditMotion = play_motion_builder_msgs::srv::EditMotion;
  using ChangeJoints = play_motion_builder_msgs::srv::ChangeJoints;

  using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;
  using ListMotions = play_motion2_msgs::srv::ListMotions;

  Ui::MotionBuilder ui_;
  QWidget * widget_;
  QMenu table_menu_;
  MotionProperties * properties_dialog_;

  rclcpp_action::Client<BuildMotion>::SharedPtr builder_client_;
  rclcpp_action::Client<RunMotion>::SharedPtr run_motion_client_;

  rclcpp::Client<ListJointGroups>::SharedPtr list_joints_client_;
  rclcpp::Client<EditMotion>::SharedPtr edit_motion_client_;
  rclcpp::Client<ChangeJoints>::SharedPtr change_joints_client_;

  rclcpp::Client<GetMotionInfo>::SharedPtr motion_info_client_;
  rclcpp::Client<ListMotions>::SharedPtr list_motions_client_;

  static const char GOTO_MENU[];
  static const char DELETE_MENU[];
  static const char SET_TO_CURRENT_MENU[];
  static const char COPY_BELOW_MENU[];
  static const char COPY_LAST_MENU[];

  bool editing_;
  bool updating_list_;
  bool motion_running_;
  std::string editing_motion_;

  EditMotion::Response::SharedPtr editMotion(
    const uint8_t action,
    const uint16_t step_id = 0u,
    const float time = 0.0);

  void changeJoints(
    const std::string & group,
    const std::vector<std::string> & joints_to_remove,
    const std::vector<std::string> & joints_to_add);

  play_motion2_msgs::msg::Motion
  getMotionInfo(const std::string & motion_key);

  std::vector<std::string> getMotionsList();

  void loadMotion(const play_motion_builder_msgs::msg::Motion & motion);

  double getJointPosition(
    const std::vector<std::string> & joints,
    const std::vector<double> & poses, const std::string & joint_header);
  void listMotion();
  void enableBtns();
  void disableBtns();
  void runDone(const rclcpp_action::ClientGoalHandle<RunMotion>::WrappedResult & result);

private slots:
  void onNewPressed();
  void onLoadPressed();
  void onCaptureClicked();
  void onPlayClicked();
  void onSaveClicked();
  void onContextMenuRequested(const QPoint & point);
  void onGotoSelected(int frame);
  void onDeleteSelected(int frame);
  void onSetToCurrentSelected(int frame);
  void onCopyBelowSelected(int frame);
  void onCopyLastSelected(int frame);
  void onGroupToggled(bool state);
  void onJointToggled(bool state);
  void onCellChanged(int row, int col);
  void onMotionStored(QString motion_name);

signals:
  void goToSelected(int row);
  void deleteSelected(int row);
  void setToCurrentSelected(int row);
  void copyBelowSelected(int row);
  void copyLastSelected(int row);
};
}  // namespace rqt_play_motion_builder

#endif  // RQT_PLAY_MOTION_BUILDER__RQT_PLAY_MOTION_BUILDER_HPP_
