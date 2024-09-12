// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "play_motion_builder.hpp"

#include <fstream>

#include "play_motion_builder_msgs/msg/frame.hpp"

#include "play_motion2_msgs/msg/motion.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"

#include "rcl_interfaces/srv/get_parameters.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace play_motion_builder
{
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

constexpr auto kTimeout = 10s;

using GetParameters = rcl_interfaces::srv::GetParameters;

ROSMotionBuilderNode::ROSMotionBuilderNode(const std::string & name)
: rclcpp::Node(name)
  , running_(false)
{
}

bool ROSMotionBuilderNode::initialize()
{
  // Wait for robot_description and robot_description_semantic to be published.
  const auto wait_for_description = [&](const std::string & topic, std::string & output) {
      std_msgs::msg::String description_msg;
      const auto subscription = this->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local(),
        [](const std_msgs::msg::String::SharedPtr) {});

      while (!rclcpp::wait_for_message(
          description_msg, subscription, this->get_node_options().context(), kTimeout))
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for %s to be published", topic.c_str());
      }
      output = description_msg.data;
    };

  wait_for_description("/robot_description", robot_description_);
  wait_for_description("/robot_description_semantic", semantic_description_);

  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Get extra joints
  auto play_motion_params_getter =
    this->create_client<GetParameters>("/play_motion2/get_parameters");

  if (!play_motion_params_getter->wait_for_service(kTimeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "rclcpp interrupted while waiting for the service.");
    } else {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Service " << play_motion_params_getter->get_service_name() << " not available.");
    }
    return false;
  }

  auto get_parameters_request = std::make_shared<GetParameters::Request>();
  get_parameters_request->names.push_back("motion_planner.exclude_from_planning_joints");
  auto param_result = play_motion_params_getter->async_send_request(get_parameters_request);
  if (rclcpp::spin_until_future_complete(
      shared_from_this(), param_result, kTimeout) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Cannot retrieve 'motion_planner.exclude_from_planning_joints' parameter from "
      "/play_motion2.");
    return false;
  }

  extra_joints_ = param_result.get()->values[0].string_array_value;

  // Set up Services
  edit_motion_server_ =
    this->create_service<EditMotion>(
    "~/edit_motion", std::bind(&ROSMotionBuilderNode::edit_motion_cb, this, _1, _2));
  store_motion_server_ =
    this->create_service<StoreMotion>(
    "~/store_motion", std::bind(&ROSMotionBuilderNode::store_motion_cb, this, _1, _2));
  change_joints_server_ =
    this->create_service<ChangeJoints>(
    "~/change_joints", std::bind(&ROSMotionBuilderNode::change_joints_cb, this, _1, _2));
  list_joints_server_ =
    this->create_service<ListJointGroups>(
    "~/list_joint_groups", std::bind(&ROSMotionBuilderNode::list_joint_groups_cb, this, _1, _2));

  // Start the action server
  build_motion_server_ = rclcpp_action::create_server<BuildMotion>(
    shared_from_this(), "~/build",
    std::bind(&ROSMotionBuilderNode::handle_build_motion_goal, this, _1, _2),
    std::bind(&ROSMotionBuilderNode::handle_build_motion_cancel, this, _1),
    std::bind(&ROSMotionBuilderNode::handle_build_motion_accepted, this, _1)
  );

  run_motion_server_ = rclcpp_action::create_server<RunMotion>(
    shared_from_this(), "~/run",
    std::bind(&ROSMotionBuilderNode::handle_run_motion_goal, this, _1, _2),
    std::bind(&ROSMotionBuilderNode::handle_run_motion_cancel, this, _1),
    std::bind(&ROSMotionBuilderNode::handle_run_motion_accepted, this, _1)
  );

  // Start the PlayMotion2 client
  play_motion_client_ = std::make_shared<play_motion2::PlayMotion2Client>("play_motion2_client");

  get_motion_info_client_ =
    this->create_client<GetMotionInfo>(
    "/play_motion2/get_motion_info", rmw_qos_profile_default, cb_group_);

  return true;
}

rclcpp_action::GoalResponse ROSMotionBuilderNode::handle_build_motion_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const BuildMotionGoal>/*goal*/)
{
  // if (running_) {
  //   return rclcpp_action::GoalResponse::REJECT;
  // }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ROSMotionBuilderNode::handle_build_motion_cancel(
  const std::shared_ptr<BuildMotionGoalHandle> goal_handle)
{
  // Prevent any further actions on the motion
  running_ = false;

  // Clear the temporal motion created if necessary
  if (motion_->getParamName() != "") {
    const auto motions_list = play_motion_client_->list_motions();
    if (std::find(
        motions_list.begin(), motions_list.end(),
        motion_->getParamName()) != motions_list.end())
    {
      play_motion_client_->remove_motion(motion_->getParamName());
    }
  }

  // Clear the motion
  motion_.reset();

  // Preempt the action
  auto result = std::make_shared<BuildMotion::Result>();
  result->ok = false;
  result->message = "Build motion process was cancelled";

  if (goal_handle->is_executing()) {
    goal_handle->canceled(result);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ROSMotionBuilderNode::handle_build_motion_accepted(
  const std::shared_ptr<BuildMotionGoalHandle> goal_handle)
{
  auto action_result = std::make_shared<BuildMotion::Result>();

  const auto goal = goal_handle->get_goal();
  if (goal->motion != "") {
    if (!get_motion_info_client_->wait_for_service(kTimeout)) {
      RCLCPP_ERROR(
        this->get_logger(), "Timeout while waiting for service %s",
        get_motion_info_client_->get_service_name());

      action_result->ok = false;
      action_result->message = "Timeout while waiting for service " +
        std::string(get_motion_info_client_->get_service_name());

      goal_handle->abort(action_result);
      return;
    }

    // Load an existing motion
    auto request = std::make_shared<GetMotionInfo::Request>();
    request->motion_key = goal->motion;
    auto motion_info_result = get_motion_info_client_->async_send_request(request);

    auto start_t = this->now();
    while (motion_info_result.wait_for(10ms) != std::future_status::ready) {
      if (this->now() - start_t > kTimeout) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Timeout while waiting for " << get_motion_info_client_->get_service_name() <<
            " result");


        action_result->ok = false;
        action_result->message = "Timeout while waiting for " +
          std::string(get_motion_info_client_->get_service_name()) + " result";

        goal_handle->abort(action_result);
        return;
      }
    }

    const auto motion_info = motion_info_result.get()->motion;
    if (!motion_info.key.empty()) {
      // Process and load motion
      motion_.reset(
        new Motion(
          this->get_logger(), motion_info,
          robot_description_, semantic_description_, extra_joints_));
      RCLCPP_INFO_STREAM(this->get_logger(), "Motion " << goal->motion << " loaded");
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Using joint group: " << motion_->getCurrentGroup());
    } else {
      auto result = std::make_shared<BuildMotion::Result>();
      result->ok = false;
      result->message = "Couldn't find motion " + goal->motion;
      goal_handle->abort(result);
      return;
    }
  } else {
    // Create a new empty motion
    motion_.reset(
      new Motion(
        this->get_logger(), robot_description_, semantic_description_, extra_joints_));
    RCLCPP_INFO(this->get_logger(), "New motion ready");
  }

  // Ready to process commands
  running_ = true;
  RCLCPP_INFO(this->get_logger(), "Motion Builder Ready");

  action_result->ok = true;
  goal_handle->succeed(action_result);
}

rclcpp_action::GoalResponse ROSMotionBuilderNode::handle_run_motion_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const RunMotionGoal>/*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ROSMotionBuilderNode::handle_run_motion_cancel(
  const std::shared_ptr<RunMotionGoalHandle>/*goal_handle*/) const
{
  RCLCPP_INFO(this->get_logger(), "Cancelling motion execution");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ROSMotionBuilderNode::handle_run_motion_accepted(
  const std::shared_ptr<RunMotionGoalHandle> goal_handle)
{
  const auto action_result = std::make_shared<RunMotion::Result>();
  if (!running_) {
    goal_handle->abort(action_result);
    return;
  }

  // Set up param name
  if (motion_->getParamName() == "") {
    motion_->setParamName();
  }

  const auto goal = goal_handle->get_goal();
  if (goal->run_mode == RunMotionGoal::RUN_MOTION) {
    const auto EPSILON = 1e-10;
    const auto motion_print = motion_->print(
      std::abs(
        goal->downshift) < EPSILON ? 1.0 : goal->downshift);
    play_motion2_msgs::msg::Motion motion_msg;
    motionToPlayMotion2Msg(motion_msg, motion_print);

    // Load motion to PlayMotion2
    if (!play_motion_client_->add_motion(motion_msg, true)) {
      goal_handle->abort(action_result);
      return;
    }
  } else if (goal->run_mode == RunMotionGoal::GO_TO_STEP) {
    auto frame = KeyFrame(rclcpp::get_logger("KeyFrame"), 0.0);  // initialize
    try {
      frame = motion_->getKeyFrame(goal->step_id);
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
      goal_handle->abort(action_result);
      return;
    }

    // Create msg with just that frame
    play_motion2_msgs::msg::Motion motion_msg;
    motion_msg.key = motion_->getParamName();
    motion_msg.joints = motion_->getJoints();
    motion_msg.times_from_start = {0.0};
    for (const auto & joint : motion_msg.joints) {
      motion_msg.positions.emplace_back(frame.getJointPosition(joint));
    }

    play_motion_client_->add_motion(motion_msg, true);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown run mode: " << goal->run_mode);
    goal_handle->abort(action_result);
    return;
  }

  // Execute the motion
  if (!play_motion_client_->run_motion(motion_->getParamName(), false)) {
    play_motion_client_->remove_motion(motion_->getParamName());
    goal_handle->abort(action_result);
    return;
  }

  // @todo: handle cancellation
  play_motion_client_->remove_motion(motion_->getParamName());
  goal_handle->succeed(action_result);
}

void ROSMotionBuilderNode::edit_motion_cb(
  EditMotion::Request::ConstSharedPtr request,
  EditMotion::Response::SharedPtr response)
{
  if (running_) {
    switch (request->action) {
      case EditMotion::Request::LIST:
        // Don't modify the motion
        break;
      case EditMotion::Request::APPEND:
      case EditMotion::Request::EDIT:
        {
          sensor_msgs::msg::JointState joint_state_msg;
          if (!rclcpp::wait_for_message(joint_state_msg, shared_from_this(), "/joint_states", 5s)) {
            response->ok = false;
            response->message = "Could not receive message from /joint_states";
            return;
          }

          if (request->action == EditMotion::Request::APPEND) {
            // Append keyframe at the end
            motion_->addKeyFrame(joint_state_msg);
          } else {
            // Update requested frame id with current position
            motion_->updateKeyFrame(joint_state_msg, request->step_id);
          }
        }
        break;
      case EditMotion::Request::COPY_AS_LAST:
      case EditMotion::Request::COPY_AS_NEXT:
        motion_->copyFrame(
          request->step_id,
          request->action == EditMotion::Request::COPY_AS_LAST ? -1 : request->step_id + 1);
        break;
      case EditMotion::Request::REMOVE:
        motion_->removeKeyFrame(request->step_id);
        break;
      case EditMotion::Request::EDIT_TIME:
        motion_->changeTime(request->step_id, request->time);
        break;
      default:
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown action code: " << request->action);
        response->ok = false;
        response->message = "Unknown action code";
        return;
    }

    // Return current motion
    motionToROSMsg(response->motion);
    response->ok = true;
  } else {
    response->ok = false;
    response->message = "No motion being built";
  }
}

void ROSMotionBuilderNode::store_motion_cb(
  StoreMotion::Request::ConstSharedPtr request,
  StoreMotion::Response::SharedPtr response) const
{
  if (running_) {
    const auto motion_print = motion_->print(
      request->meta.name, request->meta.usage,
      request->meta.description);

    YAML::Emitter e;
    e << YAML::BeginMap
      << YAML::Key << "/play_motion2"
      << YAML::Value << YAML::BeginMap
      << YAML::Key << "ros__parameters"
      << YAML::Value << YAML::BeginMap
      << YAML::Key << "motions"
      << YAML::Value << YAML::BeginMap
      << YAML::Key << request->ros_name
      << YAML::Value << motion_print
      << YAML::EndMap
      << YAML::EndMap
      << YAML::EndMap;
    RCLCPP_INFO_STREAM(this->get_logger(), "Motion: \n" << e.c_str());

    // Write file at output path
    if (request->file_path != "") {
      try {
        std::ofstream file;
        file.open(request->file_path);
        file << e.c_str();
        file.close();

        // Load to PlayMotion2
        play_motion2_msgs::msg::Motion motion_msg;
        motionToPlayMotion2Msg(motion_msg, motion_print);
        motion_msg.key = request->ros_name;
        if (!play_motion_client_->add_motion(motion_msg, true)) {
          response->ok = false;
          response->message = "Couldn't load motion to PlayMotion2";
          return;
        }
      } catch (const std::ios_base::failure & err) {
        response->ok = false;
        response->message = err.what();
        return;
      }
    }

    response->ok = true;
    response->message = std::string(e.c_str());
  } else {
    response->ok = false;
    response->message = "No motion being built, could not store.";
  }
}

void ROSMotionBuilderNode::change_joints_cb(
  ChangeJoints::Request::ConstSharedPtr request,
  ChangeJoints::Response::SharedPtr response)
{
  if (running_) {
    response->ok = true;

    // add joint position from /joint_states
    sensor_msgs::msg::JointState joint_state_msg;
    if (!rclcpp::wait_for_message(joint_state_msg, shared_from_this(), "/joint_states", 5s)) {
      response->ok = false;
      response->message = "Could not receive message from /joint_states";
      return;
    }
    // Change used group
    if (request->group != "") {
      if (!motion_->setCurrentGroup(request->group, joint_state_msg)) {
        response->ok = false;
        response->message = "Couldn't change group to " + request->group + ". ";
      }
    }
    // Set joints to remove to Unused
    if (request->joints_to_remove.size() > 0) {
      for (const auto & joint : request->joints_to_remove) {
        // Set joint to unused
        if (!motion_->setExtraJointUsedState(joint, false)) {
          response->ok = false;
          response->message += "Couldn't remove extra joint " + joint + ". ";
        }
      }
    }
    // Set joints_to_add to Used
    if (request->joints_to_add.size() > 0) {
      for (const auto & joint : request->joints_to_add) {
        // Set joint to used
        if (!motion_->setExtraJointUsedState(joint, true, joint_state_msg)) {
          response->ok = false;
          response->message += "Couldn't add extra joint " + joint + ". ";
        }
      }
    }

    // Reply
    response->current_group = motion_->getCurrentGroup();
    response->used_joints = motion_->getJoints();
  } else {
    response->ok = false;
    response->message = "No motion being built";
  }
}

void ROSMotionBuilderNode::list_joint_groups_cb(
  ListJointGroups::Request::ConstSharedPtr /*request*/,
  ListJointGroups::Response::SharedPtr response) const
{
  if (running_) {
    response->groups = motion_->getJointGroups();
    response->additional_joints = motion_->getExtraJoints();
    response->available_joints = motion_->getAvailableJoints();
  }
}

void ROSMotionBuilderNode::motionToROSMsg(play_motion_builder_msgs::msg::Motion & motion) const
{
  motion.joints = motion_->getJoints();
  motion.used_group = motion_->getCurrentGroup();
  for (unsigned int i = 0; i < motion_->size(); ++i) {
    play_motion_builder_msgs::msg::Frame f;
    for (const auto & joint : motion.joints) {
      f.pose.push_back(motion_->getKeyFrame(i).getJointPosition(joint));
    }
    f.time_from_last = motion_->getKeyFrame(i).getTime();

    motion.keyframes.push_back(f);
  }
}

void ROSMotionBuilderNode::motionToPlayMotion2Msg(
  play_motion2_msgs::msg::Motion & motion,
  const PrintMotion & motion_print) const
{
  motion.key = motion_->getParamName();
  motion.joints = motion_->getJoints();

  for (const auto & pos : motion_print.positions_) {
    motion.positions.push_back(pos);
  }
  for (const auto & time : motion_print.times_) {
    motion.times_from_start.push_back(time);
  }
}

}  // namespace play_motion_builder
