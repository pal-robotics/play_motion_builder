#include <QCheckBox>
#include <QRadioButton>
#include <QMessageBox>
#include <QInputDialog>


#include "play_motion_builder_msgs/srv/list_joint_groups.hpp"
#include "play_motion_builder_msgs/srv/change_joints.hpp"

#include "rclcpp_action/create_client.hpp"

#include "rqt_play_motion_builder/rqt_play_motion_builder.hpp"


#define JOINT_PRECISION 4
#define TIME_PRECISION 2

using std::placeholders::_1;
using namespace std::chrono_literals;
const auto kTimeout = 10s;

namespace rqt_play_motion_builder
{
const char RQTPlayMotionBuilder::GOTO_MENU[] = "Go To Position";
const char RQTPlayMotionBuilder::DELETE_MENU[] = "Delete";
const char RQTPlayMotionBuilder::SET_TO_CURRENT_MENU[] = "Recapture frame";
const char RQTPlayMotionBuilder::COPY_BELOW_MENU[] = "Copy Below";
const char RQTPlayMotionBuilder::COPY_LAST_MENU[] = "Copy as Last";

RQTPlayMotionBuilder::RQTPlayMotionBuilder()
: rqt_gui_cpp::Plugin(), widget_(0), editing_(false), updating_list_(false), editing_motion_("")
{
  setObjectName("RQTPlayMotionBuilder");
}

void RQTPlayMotionBuilder::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // Prepare group scroll area
  // Prepare scroll area
  QWidget * g_scroll_w = new QWidget();
  QVBoxLayout * g_scroll_l = new QVBoxLayout();
  g_scroll_w->setLayout(g_scroll_l);
  ui_.group_area_->setWidget(g_scroll_w);
  ui_.group_area_->setWidgetResizable(true);
  // Prepare joint scroll area
  QWidget * scroll_w = new QWidget();
  QVBoxLayout * scroll_l = new QVBoxLayout();
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
  connect(
    ui_.movement_table_, SIGNAL(cellChanged(int,int)), this,
    SLOT(onCellChanged(int,int)));

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
  connect(
    ui_.movement_table_, SIGNAL(customContextMenuRequested(const QPoint&)), this,
    SLOT(onContextMenuRequested(const QPoint&)));

  connect(this, SIGNAL(goToSelected(int)), this, SLOT(onGotoSelected(int)));
  connect(this, SIGNAL(deleteSelected(int)), this, SLOT(onDeleteSelected(int)));
  connect(this, SIGNAL(setToCurrentSelected(int)), this, SLOT(onSetToCurrentSelected(int)));
  connect(this, SIGNAL(copyBelowSelected(int)), this, SLOT(onCopyBelowSelected(int)));
  connect(this, SIGNAL(copyLastSelected(int)), this, SLOT(onCopyLastSelected(int)));

  connect(ui_.new_btn_, SIGNAL(pressed()), this, SLOT(onNewPressed()));
  connect(ui_.load_btn_, SIGNAL(pressed()), this, SLOT(onLoadPressed()));

  connect(
    properties_dialog_, SIGNAL(motionStored(QString)), this,
    SLOT(onMotionStored(QString)));

  // Connect action client
  builder_client_ = rclcpp_action::create_client<BuildMotion>(node_, "/play_motion_builder/build");
  run_motion_client_ = rclcpp_action::create_client<RunMotion>(node_, "/play_motion_builder/run");

  list_joints_client_ = node_->create_client<ListJointGroups>(
    "/play_motion_builder/list_joint_groups");
  edit_motion_client_ = node_->create_client<EditMotion>("/play_motion_builder/edit_motion");
  change_joints_client_ = node_->create_client<ChangeJoints>("/play_motion_builder/change_joints");

  motion_info_client_ = node_->create_client<GetMotionInfo>("/play_motion2/get_motion_info");
  list_motions_client_ = node_->create_client<ListMotions>("/play_motion2/list_motions");

  properties_dialog_->init(node_);
}

void RQTPlayMotionBuilder::shutdownPlugin()
{
  if (editing_) {
    builder_client_->async_cancel_all_goals();
  }
}

void RQTPlayMotionBuilder::onNewPressed()
{
  if (!builder_client_->wait_for_action_server(kTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server for building motions not available");
    return;
  }

  BuildMotion::Goal goal;  // Empty goal for new motion
  goal.motion = "";

  auto build_future = builder_client_->async_send_goal(goal);

  // wait for goal to be sent
  auto start_time = node_->now();
  while (build_future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while creating new motion");
      return;
    }
  }

  auto goal_handle = build_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal rejected by builder server");
    return;
  }

  auto result_future = builder_client_->async_get_result(goal_handle);
  start_time = node_->now();
  while (result_future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while creating new motion");
      return;
    }
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node_->get_logger(), "Error while creating new motion");
    return;
  }

  editing_ = true;
  enableBtns();

  // Load joint groups
  auto ljg = std::make_shared<ListJointGroups::Request>();
  auto ljg_future = list_joints_client_->async_send_request(ljg);

  start_time = node_->now();
  while (ljg_future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while changing joints");
      return;
    }
  }

  auto ljg_response = ljg_future.get();
  qDeleteAll(ui_.group_area_->widget()->findChildren<QWidget *>());
  for (const auto & group : ljg_response->groups) {
    // Load joint table
    QRadioButton * jsc = new QRadioButton();
    jsc->setChecked(false);
    jsc->setProperty("group_name", QString::fromStdString(group));
    connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onGroupToggled(bool)));
    jsc->setText(QString::fromStdString(group));
    ui_.group_area_->widget()->layout()->addWidget(jsc);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Group found: " << group);
  }
  qDeleteAll(ui_.joint_area_->widget()->findChildren<QWidget *>());
  for (const auto & joint : ljg_response->additional_joints) {
    // Load joint table
    QCheckBox * jsc = new QCheckBox();
    jsc->setChecked(false);
    jsc->setProperty("joint_name", QString::fromStdString(joint));
    connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onJointToggled(bool)));
    jsc->setText(QString::fromStdString(joint));
    ui_.joint_area_->widget()->layout()->addWidget(jsc);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Extra joint found: " << joint);
  }

  auto edit_result = editMotion(play_motion_builder_msgs::srv::EditMotion::Request::LIST);
  if (edit_result) {
    if (edit_result->ok) {
      loadMotion(edit_result->motion);
      editing_motion_ = "";
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR: " << edit_result->message);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "There was an error contacting the edit motion service");
  }
}

void RQTPlayMotionBuilder::loadMotion(const play_motion_builder_msgs::msg::Motion & motion)
{
  updating_list_ = true;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Begin motion loading");
  // Select group
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Using group " << motion.used_group);
  for (auto & group : ui_.group_area_->widget()->findChildren<QRadioButton *>()) {
    if (group->property("group_name").toString().toStdString() == motion.used_group) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Changing group " << motion.used_group);
      group->setChecked(true);
      continue;   // Motion will be reloaded in reaction to the group change
    }
  }

  // Generate headers
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Set headers");
  ui_.movement_table_->setColumnCount(motion.joints.size() + 1);
  // Prepare table headers
  QStringList headers;
  headers << QString("Time");
  for (const auto & joint : motion.joints) {
    headers << QString::fromStdString(joint.substr(0, joint.size() - 6));
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Joint found: " << joint);
  }
  ui_.movement_table_->setHorizontalHeaderLabels(headers);
  // Fully reload the table to get the proper column order
  ui_.movement_table_->setRowCount(0);

  // Check which extra joints are used
  for (auto & extra_joint : ui_.joint_area_->widget()->findChildren<QCheckBox *>()) {
    if (std::find(
        motion.joints.begin(), motion.joints.end(),
        extra_joint->property("joint_name").toString().toStdString()) !=
      motion.joints.end())
    {
      extra_joint->setChecked(true);
      RCLCPP_DEBUG_STREAM(
        node_->get_logger(),
        "Changing extra joint state "
          << extra_joint->property("joint_name").toString().toStdString());
    }
  }

  ui_.movement_table_->setRowCount(motion.keyframes.size());
  for (unsigned int i = 0; i < motion.keyframes.size(); ++i) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing keyframe " << i);
    // Time
    QTableWidgetItem * time_item =
      new QTableWidgetItem(QString::number(motion.keyframes[i].time_from_last, 'f', 2));
    ui_.movement_table_->setItem(i, 0, time_item);

    // Add joints in proper order
    for (int j = 1; j < ui_.movement_table_->columnCount(); ++j) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing joint " << j);
      QTableWidgetItem * joint_position_item = new QTableWidgetItem(
        QString::number(
          getJointPosition(
            motion.joints, motion.keyframes[i].pose,
            ui_.movement_table_->horizontalHeaderItem(j)->text().toStdString()),
          'f', JOINT_PRECISION));
      joint_position_item->setFlags(joint_position_item->flags() ^ Qt::ItemIsEditable);   // Non-editable

      ui_.movement_table_->setItem(i, j, joint_position_item);
    }
  }

  updating_list_ = false;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Motion loaded");
}

play_motion_builder_msgs::srv::EditMotion::Response::SharedPtr
RQTPlayMotionBuilder::editMotion(const uint8_t action, const uint16_t step_id, const float time)
{
  if (!edit_motion_client_->wait_for_service(kTimeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "There was an error contacting the edit motion service");
    }
    return nullptr;
  }
  auto em = std::make_shared<EditMotion::Request>();
  em->action = action;
  em->step_id = step_id;
  em->time = time;

  auto future = edit_motion_client_->async_send_request(em);

  const auto start_time = node_->now();
  while (future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while editing motion");
      return nullptr;
    }
  }

  auto result = future.get();
  if (!result->ok) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR: " << result->message);
  }
  return result;
}

void RQTPlayMotionBuilder::changeJoints(
  const std::string & group,
  const std::vector<std::string> & joints_to_remove,
  const std::vector<std::string> & joints_to_add)
{
  if (!change_joints_client_->wait_for_service(kTimeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(), "There was an error contacting the change joints service");
    }
    return;
  }
  auto cj = std::make_shared<ChangeJoints::Request>();
  cj->group = group;
  cj->joints_to_add = joints_to_add;
  cj->joints_to_remove = joints_to_remove;

  auto future = change_joints_client_->async_send_request(cj);
  const auto start_time = node_->now();
  while (future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while changing joints");
      return;
    }
  }

  auto result = future.get();
  if (!result->ok) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "ERROR: " << result->message);
  } else {
    listMotion();
  }
}

play_motion2_msgs::msg::Motion
RQTPlayMotionBuilder::getMotionInfo(const std::string & motion_key)
{
  play_motion2_msgs::msg::Motion motion;
  if (!motion_info_client_->wait_for_service(kTimeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "There was an error contacting with the" << motion_info_client_->get_service_name() <<
          " service");
    }
    return motion;
  }

  auto gmi = std::make_shared<GetMotionInfo::Request>();
  gmi->motion_key = motion_key;

  auto future = motion_info_client_->async_send_request(gmi);
  const auto start_time = node_->now();
  while (future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while getting motin info");
      return motion;
    }
  }

  auto result = future.get();
  return result->motion;
}

std::vector<std::string> RQTPlayMotionBuilder::getMotionsList()
{
  if (!list_motions_client_->wait_for_service(kTimeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "There was an error contacting with the" << list_motions_client_->get_service_name() <<
          " service");
    }
    return {};
  }

  auto lm = std::make_shared<ListMotions::Request>();
  auto future = list_motions_client_->async_send_request(lm);

  const auto start_time = node_->now();
  while (future.wait_for(10ms) != std::future_status::ready) {
    if (node_->now() - start_time > kTimeout) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error listing motions");
      return {};
    }
  }

  auto result = future.get();
  return result->motion_keys;
}


void RQTPlayMotionBuilder::listMotion()
{
  auto result = editMotion(EditMotion::Request::LIST);
  if (result && result->ok) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "List current motion");
    loadMotion(result->motion);
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

void RQTPlayMotionBuilder::runDone(
  const rclcpp_action::ClientGoalHandle<RunMotion>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Motion was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Motion was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }
  enableBtns();
}

double RQTPlayMotionBuilder::getJointPosition(
  const std::vector<std::string> & joints,
  const std::vector<double> & poses,
  const std::string & joint_header)
{
  for (unsigned int i = 0; i < joints.size(); ++i) {
    if (joints[i] == joint_header + "_joint") {
      return poses[i];
    }
  }

  return 0.0;
}

void RQTPlayMotionBuilder::onLoadPressed()
{
  // Open popup with options
  QStringList motions;

  const auto motions_list = getMotionsList();
  if (motions_list.empty()) {
    QMessageBox::critical(widget_, tr("Error"), tr("Could not load motions from ROS."));
    return;
  }

  for (const auto & motion : motions_list) {
    motions << tr(motion.c_str());
  }

  bool ok;
  QString motion = QInputDialog::getItem(
    widget_, tr("Load a motion"), tr("Motion:"), motions, 0, false, &ok);

  if (ok && !motion.isEmpty()) {
    if (!builder_client_->wait_for_action_server(kTimeout)) {
      RCLCPP_ERROR(node_->get_logger(), "Action server for building motions not available");
      return;
    }

    BuildMotion::Goal goal;   // Empty goal for new motion
    goal.motion = motion.toStdString();

    auto build_future = builder_client_->async_send_goal(goal);
    editing_ = true;
    enableBtns();

    // wait for goal to be sent
    auto start_time = node_->now();
    while (build_future.wait_for(10ms) != std::future_status::ready) {
      if (node_->now() - start_time > kTimeout) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while loading motion");
        return;
      }
    }

    auto goal_handle = build_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected by builder server");
      return;
    }

    auto result_future = builder_client_->async_get_result(goal_handle);
    start_time = node_->now();
    while (result_future.wait_for(10ms) != std::future_status::ready) {
      if (node_->now() - start_time > kTimeout) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while loading motion");
        return;
      }
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Motion loaded");
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error loading motion");
    }

    // Load joint groups
    auto ljg = std::make_shared<ListJointGroups::Request>();
    auto ljg_future = list_joints_client_->async_send_request(ljg);

    start_time = node_->now();
    while (ljg_future.wait_for(10ms) != std::future_status::ready) {
      if (node_->now() - start_time > kTimeout) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error while changing joints");
        return;
      }
    }
    auto ljg_response = ljg_future.get();

    // Load Group table
    qDeleteAll(ui_.group_area_->widget()->findChildren<QWidget *>());
    for (const auto & group : ljg_response->groups) {
      QRadioButton * jsc = new QRadioButton();
      jsc->setChecked(false);
      jsc->setProperty("group_name", QString::fromStdString(group));
      connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onGroupToggled(bool)));
      jsc->setText(QString::fromStdString(group));
      ui_.group_area_->widget()->layout()->addWidget(jsc);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Group found: " << group);
    }
    // Load joint table
    qDeleteAll(ui_.joint_area_->widget()->findChildren<QWidget *>());
    for (const auto & joint : ljg_response->additional_joints) {
      QCheckBox * jsc = new QCheckBox();
      jsc->setChecked(false);
      jsc->setProperty("joint_name", QString::fromStdString(joint));
      connect(jsc, SIGNAL(toggled(bool)), this, SLOT(onJointToggled(bool)));
      jsc->setText(QString::fromStdString(joint));
      ui_.joint_area_->widget()->layout()->addWidget(jsc);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Extra joint found: " << joint);
    }

    const auto em_response = editMotion(EditMotion::Request::LIST);
    if (em_response->ok) {
      loadMotion(em_response->motion);
      editing_motion_ = motion.toStdString();
    }
  }
}

void RQTPlayMotionBuilder::onCaptureClicked()
{
  auto result = editMotion(EditMotion::Request::APPEND);
  if (result && result->ok) {
    loadMotion(result->motion);
  }
}

void RQTPlayMotionBuilder::onPlayClicked()
{
  if (!motion_running_) {
    if (!run_motion_client_->wait_for_action_server(kTimeout)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Action server for running motions not available after waiting");
      return;
    }
    auto goal = RunMotion::Goal();
    goal.run_mode = RunMotion::Goal::RUN_MOTION;
    goal.downshift = ui_.downshift_->value();

    disableBtns();
    motion_running_ = true;

    auto send_goal_options = rclcpp_action::Client<RunMotion>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&RQTPlayMotionBuilder::runDone, this, _1);

    run_motion_client_->async_send_goal(goal, send_goal_options);
  } else {
    run_motion_client_->async_cancel_all_goals();   // TODO NJG not working
    motion_running_ = false;
    enableBtns();
  }
}

void RQTPlayMotionBuilder::onSaveClicked()
{
  if (!editing_motion_.empty()) {
    properties_dialog_->loadYamlName(editing_motion_);

    // Load config
    auto motion = getMotionInfo(editing_motion_);
    if (!motion.key.empty()) {
      properties_dialog_->loadMeta(motion.name, motion.usage, motion.description);
    }
  } else {
    properties_dialog_->reset();
  }
  properties_dialog_->exec();
}

void RQTPlayMotionBuilder::onContextMenuRequested(const QPoint & point)
{
  QTableWidgetItem * clicked = ui_.movement_table_->itemAt(point);
  if (clicked) {
    int row = clicked->row();
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Called on row" << row);

    QAction * selectedItem = table_menu_.exec(QCursor::pos());
    if (selectedItem) {
      RCLCPP_DEBUG_STREAM(
        node_->get_logger(),
        "Item selected is " << selectedItem->text().toStdString());

      if (selectedItem->text().toStdString() == GOTO_MENU) {
        emit goToSelected(row);
      } else if (selectedItem->text().toStdString() == DELETE_MENU) {
        emit deleteSelected(row);
      } else if (selectedItem->text().toStdString() == SET_TO_CURRENT_MENU) {
        emit setToCurrentSelected(row);
      } else if (selectedItem->text().toStdString() == COPY_BELOW_MENU) {
        emit copyBelowSelected(row);
      } else if (selectedItem->text().toStdString() == COPY_LAST_MENU) {
        emit copyLastSelected(row);
      } else {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "Selected unknown menu " << selectedItem->text().toStdString());
      }
    }
  }
}

void RQTPlayMotionBuilder::onGotoSelected(int frame)
{
  if (!run_motion_client_->wait_for_action_server(kTimeout)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Action server for running motions not available after waiting");
    return;
  }
  auto goal = RunMotion::Goal();
  goal.run_mode = RunMotion::Goal::GO_TO_STEP;
  goal.step_id = frame;

  disableBtns();
  motion_running_ = true;

  auto send_goal_options = rclcpp_action::Client<RunMotion>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&RQTPlayMotionBuilder::runDone, this, _1);

  run_motion_client_->async_send_goal(goal, send_goal_options);
}

void RQTPlayMotionBuilder::onDeleteSelected(int frame)
{
  auto result = editMotion(EditMotion::Request::REMOVE, frame);
  if (result && result->ok) {
    loadMotion(result->motion);
  }
}

void RQTPlayMotionBuilder::onSetToCurrentSelected(int frame)
{
  auto result = editMotion(EditMotion::Request::EDIT, frame);
  if (result && result->ok) {
    loadMotion(result->motion);
  }
}

void RQTPlayMotionBuilder::onCopyBelowSelected(int frame)
{
  auto result = editMotion(EditMotion::Request::COPY_AS_NEXT, frame);
  if (result && result->ok) {
    loadMotion(result->motion);
  }
}

void RQTPlayMotionBuilder::onCopyLastSelected(int frame)
{
  auto result = editMotion(EditMotion::Request::COPY_AS_LAST, frame);
  if (result && result->ok) {
    loadMotion(result->motion);
  }
}

void RQTPlayMotionBuilder::onGroupToggled(bool state)
{
  if (state && !updating_list_) {
    QString group = sender()->property("group_name").toString();
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "Change group " << group.toStdString() << " to active");

    changeJoints(group.toStdString(), {}, {});
  }
}

void RQTPlayMotionBuilder::onJointToggled(bool state)
{
  if (!updating_list_) {
    QString joint = sender()->property("joint_name").toString();
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "Change joint " << joint.toStdString() << " to "
                      << (state ? "active" : "inactive"));

    if (state) {
      changeJoints("", {}, {joint.toStdString()});
    } else {
      changeJoints("", {joint.toStdString()}, {});
    }
  }
}

void RQTPlayMotionBuilder::onCellChanged(int row, int col)
{
  if (col == 0 && !updating_list_) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Changing time");
    auto result = editMotion(
      EditMotion::Request::EDIT_TIME, row,
      ui_.movement_table_->item(row, col)->text().toDouble());
    if (result && result->ok) {
      loadMotion(result->motion);
    }
  }
}

void RQTPlayMotionBuilder::onMotionStored(QString motion_name)
{
  editing_motion_ = motion_name.toStdString();
  properties_dialog_->close();
}

}  // namespace rqt_play_motion_builder

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rqt_play_motion_builder::RQTPlayMotionBuilder, rqt_gui_cpp::Plugin)
