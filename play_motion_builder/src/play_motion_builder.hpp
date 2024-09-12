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

#ifndef PLAY_MOTION_BUILDER_HPP_
#define PLAY_MOTION_BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "play_motion_builder_msgs/action/build_motion.hpp"
#include "play_motion_builder_msgs/action/run_motion.hpp"
#include "play_motion_builder_msgs/srv/change_joints.hpp"
#include "play_motion_builder_msgs/srv/edit_motion.hpp"
#include "play_motion_builder_msgs/srv/list_joint_groups.hpp"
#include "play_motion_builder_msgs/srv/store_motion.hpp"

#include "play_motion_builder/motion_model.hpp"

#include "play_motion2/client.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/msg/motion.hpp"
#include "play_motion2_msgs/srv/get_motion_info.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"

#include "std_msgs/msg/string.hpp"

namespace play_motion_builder
{
class ROSMotionBuilderNode : public rclcpp::Node
{
  using BuildMotion = play_motion_builder_msgs::action::BuildMotion;
  using BuildMotionGoal = BuildMotion::Goal;
  using BuildMotionGoalHandle = rclcpp_action::ServerGoalHandle<BuildMotion>;

  using RunMotion = play_motion_builder_msgs::action::RunMotion;
  using RunMotionGoal = RunMotion::Goal;
  using RunMotionGoalHandle = rclcpp_action::ServerGoalHandle<RunMotion>;

  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;

  using EditMotion = play_motion_builder_msgs::srv::EditMotion;
  using StoreMotion = play_motion_builder_msgs::srv::StoreMotion;
  using ChangeJoints = play_motion_builder_msgs::srv::ChangeJoints;
  using ListJointGroups = play_motion_builder_msgs::srv::ListJointGroups;

  using GetMotionInfo = play_motion2_msgs::srv::GetMotionInfo;

public:
  explicit ROSMotionBuilderNode(const std::string & name);
  bool initialize();

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp_action::Server<BuildMotion>::SharedPtr build_motion_server_;
  rclcpp_action::Server<RunMotion>::SharedPtr run_motion_server_;
  std::shared_ptr<play_motion2::PlayMotion2Client> play_motion_client_;

  rclcpp::Service<EditMotion>::SharedPtr edit_motion_server_;
  rclcpp::Service<StoreMotion>::SharedPtr store_motion_server_;
  rclcpp::Service<ChangeJoints>::SharedPtr change_joints_server_;
  rclcpp::Service<ListJointGroups>::SharedPtr list_joints_server_;
  rclcpp::Client<GetMotionInfo>::SharedPtr get_motion_info_client_;
  bool running_;

  std::string robot_description_;
  std::string semantic_description_;
  std::vector<std::string> extra_joints_;

  std::unique_ptr<Motion> motion_;

  // Action callbacks
  rclcpp_action::GoalResponse handle_build_motion_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BuildMotionGoal> goal);

  rclcpp_action::CancelResponse handle_build_motion_cancel(
    const std::shared_ptr<BuildMotionGoalHandle> goal_handle);

  void handle_build_motion_accepted(const std::shared_ptr<BuildMotionGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handle_run_motion_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunMotionGoal> goal);

  rclcpp_action::CancelResponse handle_run_motion_cancel(
    const std::shared_ptr<RunMotionGoalHandle> goal_handle) const;

  void handle_run_motion_accepted(const std::shared_ptr<RunMotionGoalHandle> goal_handle);

  // Service callbacks
  void edit_motion_cb(
    EditMotion::Request::ConstSharedPtr request,
    EditMotion::Response::SharedPtr response);

  void store_motion_cb(
    StoreMotion::Request::ConstSharedPtr request,
    StoreMotion::Response::SharedPtr response) const;

  void change_joints_cb(
    ChangeJoints::Request::ConstSharedPtr request,
    ChangeJoints::Response::SharedPtr response);

  void list_joint_groups_cb(
    ListJointGroups::Request::ConstSharedPtr request,
    ListJointGroups::Response::SharedPtr response) const;

  // Utility methods
  void motionToROSMsg(play_motion_builder_msgs::msg::Motion & motion) const;
  void motionToPlayMotion2Msg(
    play_motion2_msgs::msg::Motion & motion,
    const PrintMotion & motion_print) const;
};

}  // namespace play_motion_builder

#endif  // PLAY_MOTION_BUILDER_HPP_
