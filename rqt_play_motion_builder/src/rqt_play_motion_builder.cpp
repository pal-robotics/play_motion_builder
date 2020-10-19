#include <rqt_play_motion_builder/rqt_play_motion_builder.h>

#include <play_motion_builder_msgs/ListJointGroups.h>
#include <play_motion_builder_msgs/ChangeJoints.h>

#include <QCheckBox>
#include <QRadioButton>
#include <QMessageBox>
#include <QInputDialog>

#include <pluginlib/class_list_macros.h>

#define JOINT_PRECISION 4
#define TIME_PRECISION 2

namespace pal
{
const std::string RQTPlayMotionBuilder::GOTO_MENU = "Go To Position";
const std::string RQTPlayMotionBuilder::DELETE_MENU = "Delete";
const std::string RQTPlayMotionBuilder::SET_TO_CURRENT_MENU = "Recapture frame";
const std::string RQTPlayMotionBuilder::COPY_BELOW_MENU = "Copy Below";
const std::string RQTPlayMotionBuilder::COPY_LAST_MENU = "Copy as Last";

RQTPlayMotionBuilder::RQTPlayMotionBuilder()
  : rqt_gui_cpp::Plugin(), widget_(0), editing_(false), updating_list_(false), editing_motion_("")
{
  setObjectName("RQTPlayMotionBuilder");
}

void RQTPlayMotionBuilder::initPlugin(qt_gui_cpp::PluginContext &context)
{
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // Prepare group scroll area
  // Prepare scroll area
  QWidget *g_scroll_w = new QWidget();
  QVBoxLayout *g_scroll_l = new QVBoxLayout();
  g_scroll_w->setLayout(g_scroll_l);
  ui_.group_area_->setWidget(g_scroll_w);
  ui_.group_area_->setWidgetResizable(true);
  // Prepare joint scroll area
  QWidget *scroll_w = new QWidget();
  QVBoxLayout *scroll_l = new QVBoxLayout();
  scroll_w->setLayout(scroll_l);
  ui_.joint_area_->setWidget(scroll_w);
  ui_.joint_area_->setWidgetResizable(true);
  // add widget to the user interface
  context.addWidget(widget_);

  // Setup properties dialog
  properties_dialog_ = new MotionProperties(widget_);

  // Setup buttons
  connect(ui_.capture_btn_, SIGNAL(pressed()), this, SLOT(onCaptureClicked()));
  connect(ui_.play_btn_, SIGNAL(pressed()), this, SLOT(onPlayClicked()));
  connect(ui_.save_btn_, SIGNAL(pressed()), this, SLOT(onSaveClicked()));
  connect(ui_.movement_table_, SIGNAL(cellChanged(int, int)), this,
          SLOT(onCellChanged(int, int)));

  // Set up context menu
  ui_.movement_table_->setContextMenuPolicy(Qt::CustomContextMenu);
  // Setup menus
  table_menu_.addAction(QString::fromStdString(GOTO_MENU));
  table_menu_.addAction(QString::fromStdString(SET_TO_CURRENT_MENU));
  table_menu_.addSeparator();
  table_menu_.addAction(QString::fromStdString(COPY_BELOW_MENU));
  table_menu_.addAction(QString::fromStdString(COPY_LAST_MENU));
  table_menu_.addSeparator();
  table_menu_.addAction(QString::fromStdString(DELETE_MENU));
  connect(ui_.movement_table_, SIGNAL(customContextMenuRequested(const QPoint &)), this,
          SLOT(onContextMenuRequested(const QPoint &)));

  connect(this, SIGNAL(goToSelected(int)), this, SLOT(onGotoSelected(int)));
  connect(this, SIGNAL(deleteSelected(int)), this, SLOT(onDeleteSelected(int)));
  connect(this, SIGNAL(setToCurrentSelected(int)), this, SLOT(onSetToCurrentSelected(int)));
  connect(this, SIGNAL(copyBelowSelected(int)), this, SLOT(onCopyBelowSelected(int)));
  connect(this, SIGNAL(copyLastSelected(int)), this, SLOT(onCopyLastSelected(int)));

  connect(ui_.new_btn_, SIGNAL(pressed()), this, SLOT(onNewPressed()));
  connect(ui_.load_btn_, SIGNAL(pressed()), this, SLOT(onLoadPressed()));

  connect(properties_dialog_, SIGNAL(motionStored(QString)), this,
          SLOT(onMotionStored(QString)));

  // Connect action client
  builder_client_.reset(new BMAC(getNodeHandle(), "/play_motion_builder_node/build"));
  run_motion_client_.reset(new RMAC(getNodeHandle(), "/play_motion_builder_node/run"));
  list_joints_client_ = getNodeHandle().serviceClient<play_motion_builder_msgs::ListJointGroups>(
      "/play_motion_builder_node/list_joint_groups");
  edit_motion_client_ = getNodeHandle().serviceClient<play_motion_builder_msgs::EditMotion>(
      "/play_motion_builder_node/edit_motion");
  change_joints_client_ = getNodeHandle().serviceClient<play_motion_builder_msgs::ChangeJoints>(
      "/play_motion_builder_node/change_joints");
  properties_dialog_->init(getNodeHandle());
}

void RQTPlayMotionBuilder::shutdownPlugin()
{
  if (editing_)
  {
    builder_client_->cancelAllGoals();
  }
}

void RQTPlayMotionBuilder::onNewPressed()
{
  if (!builder_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR_STREAM("Couldn't contact builder server");
  }
  else
  {
    play_motion_builder_msgs::BuildMotionGoal goal;  // Empty goal for new motion
    builder_client_->sendGoal(goal);
    editing_ = true;
    enableBtns();
    ros::Duration(0.1).sleep();  // Wait for a bit

    play_motion_builder_msgs::ListJointGroups ljg_service;
    if (list_joints_client_.call(ljg_service))
    {
      qDeleteAll(ui_.group_area_->widget()->findChildren<QWidget *>());
      // Load Group table
      for (const auto &group : ljg_service.response.groups)
      {
        // Load joint table
        QRadioButton *jsc = new QRadioButton();
        jsc->setChecked(false);
        jsc->setProperty("group_name", QString::fromStdString(group));
        connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onGroupToggled(bool)));
        jsc->setText(QString::fromStdString(group));
        ui_.group_area_->widget()->layout()->addWidget(jsc);
        ROS_DEBUG_STREAM("Group found: " << group);
      }
      qDeleteAll(ui_.joint_area_->widget()->findChildren<QWidget *>());
      for (const auto &joint : ljg_service.response.additional_joints)
      {
        // Load joint table
        QCheckBox *jsc = new QCheckBox();
        jsc->setChecked(false);
        jsc->setProperty("joint_name", QString::fromStdString(joint));
        connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onJointToggled(bool)));
        jsc->setText(QString::fromStdString(joint));
        ui_.joint_area_->widget()->layout()->addWidget(jsc);
        ROS_DEBUG_STREAM("Extra joint found: " << joint);
      }

      play_motion_builder_msgs::EditMotion em;
      em.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
      if (edit_motion_client_.call(em))
      {
        if (em.response.ok)
        {
          loadMotion(em.response.motion);
          editing_motion_ = "";
        }
        else
        {
          ROS_ERROR_STREAM("ERROR: " << em.response.message);
        }
      }
      else
      {
        ROS_ERROR_STREAM("There was an error contacting the edit motion service");
      }
    }
  }
}

void RQTPlayMotionBuilder::loadMotion(const play_motion_builder_msgs::Motion &motion)
{
  updating_list_ = true;
  ROS_DEBUG_STREAM("Begin motion loading");
  // Select group
  ROS_DEBUG_STREAM("Using group " << motion.used_group);
  for (auto &group : ui_.group_area_->widget()->findChildren<QRadioButton *>())
  {
    if (group->property("group_name").toString().toStdString() == motion.used_group)
    {
      ROS_DEBUG_STREAM("Changing group " << motion.used_group);
      group->setChecked(true);
      continue;  // Motion will be reloaded in reaction to the group change
    }
  }

  // Generate headers
  ROS_DEBUG_STREAM("Set headers");
  ui_.movement_table_->setColumnCount(motion.joints.size() + 1);
  // Prepare table headers
  QStringList headers;
  headers << QString("Time");
  for (const auto &joint : motion.joints)
  {
    headers << QString::fromStdString(joint.substr(0, joint.size() - 6));
    ROS_DEBUG_STREAM("Joint found: " << joint);
  }
  ui_.movement_table_->setHorizontalHeaderLabels(headers);
  // Fully reload the table to get the proper column order
  ui_.movement_table_->setRowCount(0);

  // Check which extra joints are used
  for (auto &extra_joint : ui_.joint_area_->widget()->findChildren<QCheckBox *>())
  {
    if (std::find(motion.joints.begin(), motion.joints.end(),
                  extra_joint->property("joint_name").toString().toStdString()) !=
        motion.joints.end())
    {
      extra_joint->setChecked(true);
      ROS_DEBUG_STREAM("Changing extra joint state "
                       << extra_joint->property("joint_name").toString().toStdString());
    }
  }

  ui_.movement_table_->setRowCount(motion.keyframes.size());
  for (unsigned int i = 0; i < motion.keyframes.size(); ++i)
  {
    ROS_DEBUG_STREAM("Processing keyframe " << i);
    // Time
    QTableWidgetItem *time_item =
        new QTableWidgetItem(QString::number(motion.keyframes[i].time_from_last, 'f', 2));
    ui_.movement_table_->setItem(i, 0, time_item);


    // Add joints in proper order
    for (int j = 1; j < ui_.movement_table_->columnCount(); ++j)
    {
      ROS_DEBUG_STREAM("Processing joint " << j);
      QTableWidgetItem *joint_position_item = new QTableWidgetItem(QString::number(
          getJointPosition(motion.joints, motion.keyframes[i].pose,
                           ui_.movement_table_->horizontalHeaderItem(j)->text().toStdString()),
          'f', JOINT_PRECISION));
      joint_position_item->setFlags(joint_position_item->flags() ^ Qt::ItemIsEditable);  // Non-editable

      ui_.movement_table_->setItem(i, j, joint_position_item);
    }
  }

  updating_list_ = false;
  ROS_DEBUG_STREAM("Motion loaded");
}

void RQTPlayMotionBuilder::listMotion()
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      ROS_DEBUG_STREAM("List current motion");
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("There was an error contacting the edit motion service");
  }
}

void RQTPlayMotionBuilder::enableBtns()
{
  ui_.play_btn_->setText("Play");
  ui_.play_btn_->setEnabled(true);
  ui_.downshift_->setEnabled(true);
  ui_.capture_btn_->setEnabled(true);
  ui_.save_btn_->setEnabled(true);
}

void RQTPlayMotionBuilder::disableBtns()
{
  ui_.play_btn_->setText("Stop");
  ui_.downshift_->setEnabled(false);
  ui_.capture_btn_->setEnabled(false);
  ui_.save_btn_->setEnabled(false);
}

void RQTPlayMotionBuilder::runDone(const actionlib::SimpleClientGoalState &,
                               const play_motion_builder_msgs::RunMotionResultConstPtr &)
{
  enableBtns();
}

double RQTPlayMotionBuilder::getJointPosition(const std::vector<std::string> &joints,
                                          const std::vector<double> &poses,
                                          const std::string &joint_header)
{
  for (unsigned int i = 0; i < joints.size(); ++i)
  {
    if (joints[i] == joint_header + "_joint")
      return poses[i];
  }

  return 0.0;
}

void RQTPlayMotionBuilder::onLoadPressed()
{
  // Open popup with options
  QStringList motions;
  XmlRpc::XmlRpcValue param;
  getNodeHandle().getParam("/play_motion/motions", param);
  if (param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    QMessageBox::critical(widget_, tr("Error"), tr("Could not load motions from ROS."));
    return;
  }

  for (auto it : param)
  {
    motions << tr(it.first.c_str());
  }

  bool ok;
  QString motion = QInputDialog::getItem(widget_, tr("Load a motion"), tr("Motion:"),
                                         motions, 0, false, &ok);

  if (ok && !motion.isEmpty())
  {
    if (!builder_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR_STREAM("Couldn't contact builder server");
    }
    else
    {
      play_motion_builder_msgs::BuildMotionGoal goal;  // Empty goal for new motion
      goal.motion = motion.toStdString();
      builder_client_->sendGoal(goal);
      editing_ = true;
      enableBtns();
      ros::Duration(0.1).sleep();  // Wait for a bit

      play_motion_builder_msgs::ListJointGroups ljg_service;
      if (list_joints_client_.call(ljg_service))
      {
        // Load Group table
        qDeleteAll(ui_.group_area_->widget()->findChildren<QWidget *>());
        for (const auto &group : ljg_service.response.groups)
        {
          QRadioButton *jsc = new QRadioButton();
          jsc->setChecked(false);
          jsc->setProperty("group_name", QString::fromStdString(group));
          connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onGroupToggled(bool)));
          jsc->setText(QString::fromStdString(group));
          ui_.group_area_->widget()->layout()->addWidget(jsc);
          ROS_INFO_STREAM("Group found: " << group);
        }
        // Load joint table
        qDeleteAll(ui_.joint_area_->widget()->findChildren<QWidget *>());
        for (const auto &joint : ljg_service.response.additional_joints)
        {
          QCheckBox *jsc = new QCheckBox();
          jsc->setChecked(false);
          jsc->setProperty("joint_name", QString::fromStdString(joint));
          connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onJointToggled(bool)));
          jsc->setText(QString::fromStdString(joint));
          ui_.joint_area_->widget()->layout()->addWidget(jsc);
          ROS_INFO_STREAM("Extra joint found: " << joint);
        }

        play_motion_builder_msgs::EditMotion em;
        em.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
        if (edit_motion_client_.call(em))
        {
          if (em.response.ok)
          {
            loadMotion(em.response.motion);
            editing_motion_ = motion.toStdString();
          }
          else
          {
            ROS_ERROR_STREAM("ERROR: " << em.response.message);
          }
        }
        else
        {
          ROS_ERROR_STREAM("There was an error contacting the edit motion service");
        }
      }
    }
  }
}

void RQTPlayMotionBuilder::onCaptureClicked()
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::APPEND;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error calling the edit_motion service to append keyframe");
  }
}

void RQTPlayMotionBuilder::onPlayClicked()
{
  if (!motion_running_)
  {
    play_motion_builder_msgs::RunMotionGoal goal;
    goal.run_mode = play_motion_builder_msgs::RunMotionGoal::RUN_MOTION;
    goal.downshift = ui_.downshift_->value();

    disableBtns();
    motion_running_ = true;
    run_motion_client_->sendGoal(goal, boost::bind(&RQTPlayMotionBuilder::runDone, this, _1,
                                                   _2));  // Add done callback to unlock
  }
  else
  {
    run_motion_client_->cancelAllGoals();
    motion_running_ = false;
  }
}

void RQTPlayMotionBuilder::onSaveClicked()
{
  if (!editing_motion_.empty())
  {
    properties_dialog_->loadYamlName(editing_motion_);
    // Load config
    XmlRpc::XmlRpcValue param;
    if (getNodeHandle().getParam("/play_motion/motions/" + editing_motion_ + "/meta", param))
    {
      properties_dialog_->loadMeta(static_cast<std::string>(param["name"]),
                                   static_cast<std::string>(param["usage"]),
                                   static_cast<std::string>(param["description"]));
    }
  }
  else
  {
    properties_dialog_->reset();
  }
  properties_dialog_->exec();
}

void RQTPlayMotionBuilder::onContextMenuRequested(const QPoint &point)
{
  QTableWidgetItem *clicked = ui_.movement_table_->itemAt(point);
  if (clicked)
  {
    int row = clicked->row();
    ROS_DEBUG_STREAM("Called on row" << row);

    QAction *selectedItem = table_menu_.exec(QCursor::pos());
    if (selectedItem)
    {
      ROS_DEBUG_STREAM("Item selected is " << selectedItem->text().toStdString());

      if (selectedItem->text().toStdString() == GOTO_MENU)
      {
        emit goToSelected(row);
      }
      else if (selectedItem->text().toStdString() == DELETE_MENU)
      {
        emit deleteSelected(row);
      }
      else if (selectedItem->text().toStdString() == SET_TO_CURRENT_MENU)
      {
        emit setToCurrentSelected(row);
      }
      else if (selectedItem->text().toStdString() == COPY_BELOW_MENU)
      {
        emit copyBelowSelected(row);
      }
      else if (selectedItem->text().toStdString() == COPY_LAST_MENU)
      {
        emit copyLastSelected(row);
      }
      else
      {
        ROS_ERROR_STREAM("Selected unknown menu " << selectedItem->text().toStdString());
      }
    }
  }
}
void RQTPlayMotionBuilder::onGotoSelected(int frame)
{
  play_motion_builder_msgs::RunMotionGoal goal;
  goal.run_mode = play_motion_builder_msgs::RunMotionGoal::GO_TO_STEP;
  goal.step_id = frame;

  disableBtns();
  motion_running_ = true;
  run_motion_client_->sendGoal(goal, boost::bind(&RQTPlayMotionBuilder::runDone, this, _1, _2));  // Add done callback to unlock
}

void RQTPlayMotionBuilder::onDeleteSelected(int frame)
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::REMOVE;
  em.request.step_id = frame;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error calling the edit_motion service to edit keyframe");
  }
}
void RQTPlayMotionBuilder::onSetToCurrentSelected(int frame)
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::EDIT;
  em.request.step_id = frame;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error calling the edit_motion service to edit keyframe");
  }
}
void RQTPlayMotionBuilder::onCopyBelowSelected(int frame)
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::COPY_AS_NEXT;
  em.request.step_id = frame;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error calling the edit_motion service to copy keyframe");
  }
}
void RQTPlayMotionBuilder::onCopyLastSelected(int frame)
{
  play_motion_builder_msgs::EditMotion em;
  em.request.action = play_motion_builder_msgs::EditMotion::Request::COPY_AS_LAST;
  em.request.step_id = frame;
  if (edit_motion_client_.call(em))
  {
    if (em.response.ok)
    {
      loadMotion(em.response.motion);
    }
    else
    {
      ROS_ERROR_STREAM("ERROR: " << em.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Error calling the edit_motion service to copy-as-last keyframe");
  }
}

void RQTPlayMotionBuilder::onGroupToggled(bool state)
{
  if (state && !updating_list_)
  {
    QString group = sender()->property("group_name").toString();
    ROS_DEBUG_STREAM("Change group " << group.toStdString() << " to active");

    play_motion_builder_msgs::ChangeJoints cj;
    cj.request.group = group.toStdString();
    if (change_joints_client_.call(cj))
    {
      if (cj.response.ok)
      {
        listMotion();
      }
      else
      {
        ROS_ERROR_STREAM("ERROR: " << cj.response.message);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Error calling the edit_motion service to copy-as-last keyframe");
    }
  }
}

void RQTPlayMotionBuilder::onJointToggled(bool state)
{
  if (!updating_list_)
  {
    QString joint = sender()->property("joint_name").toString();
    ROS_DEBUG_STREAM("Change joint " << joint.toStdString() << " to "
                                     << (state ? "active" : "inactive"));

    play_motion_builder_msgs::ChangeJoints cj;
    if (state)
      cj.request.joints_to_add.push_back(joint.toStdString());
    else
      cj.request.joints_to_remove.push_back(joint.toStdString());

    if (change_joints_client_.call(cj))
    {
      if (cj.response.ok)
      {
        listMotion();
      }
      else
      {
        ROS_ERROR_STREAM("ERROR: " << cj.response.message);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Error calling the edit_motion service to copy-as-last keyframe");
    }
  }
}

void RQTPlayMotionBuilder::onCellChanged(int row, int col)
{
  if (col == 0 && !updating_list_)
  {
    ROS_DEBUG_STREAM("Changing time");
    play_motion_builder_msgs::EditMotion em;
    em.request.action = play_motion_builder_msgs::EditMotion::Request::EDIT_TIME;
    em.request.step_id = row;
    em.request.time = ui_.movement_table_->item(row, col)->text().toDouble();

    if (edit_motion_client_.call(em))
    {
      if (em.response.ok)
      {
        loadMotion(em.response.motion);
      }
      else
      {
        ROS_ERROR_STREAM("ERROR: " << em.response.message);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Error calling the edit_motion service to copy-as-last keyframe");
    }
  }
}

void RQTPlayMotionBuilder::onMotionStored(QString motion_name)
{
  editing_motion_ = motion_name.toStdString();
  properties_dialog_->close();
}

}  // namespace pal

PLUGINLIB_EXPORT_CLASS(pal::RQTPlayMotionBuilder, rqt_gui_cpp::Plugin)
