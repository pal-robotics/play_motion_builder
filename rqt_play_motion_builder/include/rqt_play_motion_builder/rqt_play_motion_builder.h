#ifndef RQT_MOTION_BUILDER_H
#define RQT_MOTION_BUILDER_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_rqt_play_motion_builder.h>
#include <rqt_play_motion_builder/properties_dialog.h>

#include <play_motion_builder_msgs/BuildMotionAction.h>
#include <play_motion_builder_msgs/EditMotion.h>
#include <play_motion_builder_msgs/RunMotionAction.h>

#include <actionlib/client/simple_action_client.h>

#include <QMenu>

namespace pal
{
class RQTPlayMotionBuilder : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  RQTPlayMotionBuilder();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings&, qt_gui_cpp::Settings&) const override
  {
  }
  virtual void restoreSettings(const qt_gui_cpp::Settings&, const qt_gui_cpp::Settings&) override
  {
  }

private:
  typedef actionlib::SimpleActionClient<play_motion_builder_msgs::BuildMotionAction> BMAC;
  typedef actionlib::SimpleActionClient<play_motion_builder_msgs::RunMotionAction> RMAC;

  Ui::MotionBuilder ui_;
  QWidget* widget_;
  QMenu table_menu_;
  MotionProperties* properties_dialog_;

  std::unique_ptr<BMAC> builder_client_;
  std::unique_ptr<RMAC> run_motion_client_;
  ros::ServiceClient list_joints_client_;
  ros::ServiceClient edit_motion_client_;
  ros::ServiceClient change_joints_client_;

  const static std::string GOTO_MENU;
  const static std::string DELETE_MENU;
  const static std::string SET_TO_CURRENT_MENU;
  const static std::string COPY_BELOW_MENU;
  const static std::string COPY_LAST_MENU;

  bool editing_;
  bool updating_list_;
  bool motion_running_;
  std::string editing_motion_;

  void loadMotion(const play_motion_builder_msgs::Motion& motion);
  double getJointPosition(const std::vector<std::string>& joints,
                          const std::vector<double>& poses, const std::string& joint_header);
  void listMotion();
  void enableBtns();
  void disableBtns();
  void runDone(const actionlib::SimpleClientGoalState& state,
               const play_motion_builder_msgs::RunMotionResultConstPtr& result);

private slots:
  void onNewPressed();
  void onLoadPressed();
  void onCaptureClicked();
  void onPlayClicked();
  void onSaveClicked();
  void onContextMenuRequested(const QPoint& point);
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
}  // namespace pal

#endif /* RQT_MOTION_BUILDER_H */
