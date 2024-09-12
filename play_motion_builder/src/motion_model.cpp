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

#include "play_motion_builder/motion_model.hpp"

#include <stdio.h>

#include <fstream>
#include <cstdio>
#include <thread>
#include <unordered_set>

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"

#include "rclcpp/logging.hpp"

namespace play_motion_builder
{
const char PrintMotion::TIME_KEY[] = "times_from_start";
const char PrintMotion::POSITIONS_KEY[] = "positions";
const char PrintMotion::JOINTS_KEY[] = "joints";
const char PrintMeta::META_KEY[] = "meta";
const char PrintMeta::NAME_KEY[] = "name";
const char PrintMeta::USAGE_KEY[] = "usage";
const char PrintMeta::DESC_KEY[] = "description";

KeyFrame::KeyFrame(const rclcpp::Logger & logger, float time_increment)
: logger_(logger)
  , time_increment_(time_increment)
{
  RCLCPP_INFO_STREAM(logger_, "frame created");
}

KeyFrame::KeyFrame(const KeyFrame & k)
: logger_(k.logger_)
  , joints_(k.joints_)
  , time_increment_(k.time_increment_)
{
}

void KeyFrame::addPosition(const std::string & name, double position)
{
  joints_.push_back(JointPosition(name, position));
}

double KeyFrame::getJointPosition(const std::string & joint) const
{
  for (auto joint_position : joints_) {
    if (joint_position.joint_name_ == joint) {
      return joint_position.position_;
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

void KeyFrame::cleanUnused(const std::map<std::string, bool> & used_joints)
{
  for (unsigned int i = joints_.size(); i > 0; --i) {  // Iterate backwards and remove unused
    try {
      if (!used_joints.at(joints_[i - 1].joint_name_)) {
        joints_.erase(joints_.begin() + i - 1);
      }
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(logger_, "Key: " << joints_[i - 1].joint_name_ << " is unknown");
      throw e;
    }
  }
}

Point KeyFrame::get_point(
  double basetime, double downshift,
  const std::vector<std::string> & names) const
{
  Point p;
  p.time_from_start_ = basetime + (downshift * time_increment_);

  for (auto joint_name : names) {
    p.positions_.push_back(getJointPosition(joint_name));
  }

  return p;
}

PrintMotion KeyFrame::print(const std::vector<std::string> & names) const
{
  PrintMotion pm;

  for (auto joint : joints_) {
    if (std::find(names.begin(), names.end(), joint.joint_name_) != names.end()) {
      pm.joints_.push_back(joint.joint_name_);
      pm.positions_.push_back(joint.position_);
    }
  }

  pm.times_ = {0.0};
  pm.meta_.print_ = false;
  return pm;
}

const float Motion::DEFAULT_TIME = 5.0;

Motion::Motion(
  const rclcpp::Logger & logger,
  const std::string & robot_description, const std::string & robot_description_semantic,
  const std::vector<std::string> & extra_joints)
: logger_(logger)
  , tmp_name_("")
{
  // Collect "out of group" joints
  for (const auto & joint : extra_joints) {
    extra_joints_[joint] = true;
  }

  // Load groups
  if (robot_description != "" && robot_description_semantic != "") {
    setMotionGroups(robot_description, robot_description_semantic);

    // Pre-select biggest group
    uint size = 0;
    for (const auto & group : joint_groups_) {
      if (group.second.size() >= size) {
        group_used_ = group.first;
        size = group.second.size();
      }
    }
  }
}

Motion::Motion(
  const rclcpp::Logger & logger,
  const play_motion2_msgs::msg::Motion & motion_cfg,
  const std::string & robot_description,
  const std::string & robot_description_semantic,
  const std::vector<std::string> & extra_joints)
: Motion(logger, robot_description, robot_description_semantic, extra_joints)
{
  // Load used joints
  RCLCPP_INFO(logger_, "Joint used:");
  std::vector<std::string> names;
  for (const auto & joint : motion_cfg.joints) {
    names.push_back(joint);
    RCLCPP_INFO_STREAM(logger_, joint);
  }

  // Set state for extra joints
  for (auto & joint : extra_joints_) {
    joint.second = std::find(names.begin(), names.end(), joint.first) != names.end();
  }

  // Find used group (biggest group with all joints used)
  std::string group_used = "";
  uint size = 0;
  for (const auto & group : joint_groups_) {
    // Used group can't have more joints, only check if bigger than found
    if (group.second.size() <= names.size() && group.second.size() > size) {
      // Check if all joints are used
      bool all_used = true;
      for (const std::string & joint : group.second) {
        if (std::find(names.begin(), names.end(), joint) == names.end()) {
          RCLCPP_INFO_STREAM(
            logger_,
            "Joint " << joint << " from group " << group.first << " not in use");
          all_used = false;
          break;
        }
      }

      // If all used, choose this one
      if (all_used) {
        size = group.second.size();
        group_used = group.first;
        RCLCPP_INFO_STREAM(logger_, "Found group candidate " << group_used);
      }
    } else {
      RCLCPP_INFO_STREAM(
        logger_,
        "Group " << group.first << " discarded (" << group.second.size()
                 << "<=" << names.size() << " && " << group.second.size()
                 << ">" << size << ")");
    }
  }
  group_used_ = group_used;

  // Populate motion
  auto last_time = 0.0f;
  auto offset = 0u;
  for (auto i = 0u; i < motion_cfg.times_from_start.size(); ++i) {
    KeyFrame k(logger_, motion_cfg.times_from_start[i] - last_time);
    // Load joints in order
    for (auto j = 0u; j < motion_cfg.joints.size(); ++j) {
      k.addPosition(names[j], motion_cfg.positions[j + offset]);
    }
    offset += motion_cfg.joints.size();
    keyframes_.push_back(k);
    last_time += k.getTime();
  }
}

void Motion::setMotionGroups(
  const std::string & robot_description,
  const std::string & robot_description_semantic)
{
  // Load XML robot's description
  boost::property_tree::ptree tree_robot;
  std::stringstream ss_robot(robot_description);
  boost::property_tree::read_xml(ss_robot, tree_robot);

  // Find all actuable joints
  std::unordered_set<std::string> actuable_joints;
  for (boost::property_tree::ptree::value_type & joint : tree_robot.get_child("robot")) {
    if (joint.first != "joint") {   // Ignore others
      continue;
    }

    // Get joint name
    std::string joint_name;
    bool is_actuable = true;
    for (boost::property_tree::ptree::value_type & joint_child : joint.second) {
      if (joint_child.first == "<xmlattr>") {
        for (boost::property_tree::ptree::value_type & joint_att : joint_child.second) {
          if (joint_att.first == "name") {
            joint_name = joint_att.second.data();
          } else if (joint_att.first == "type" && joint_att.second.data() == "fixed") {
            is_actuable = false;
          }
        }
      }
    }

    if (is_actuable) {
      actuable_joints.insert(joint_name);
      RCLCPP_DEBUG_STREAM(logger_, "Actuable joint: " << joint_name);
    }
  }

  // Load XML groups description
  boost::property_tree::ptree tree_semantic;
  std::stringstream ss_semantic(robot_description_semantic);
  boost::property_tree::read_xml(ss_semantic, tree_semantic);

  // Load groups
  std::vector<std::pair<std::string, std::string>> pending_groups_;
  for (boost::property_tree::ptree::value_type & group : tree_semantic.get_child("robot")) {
    if (group.first != "group") {   // Ignore others
      continue;
    }

    // Get group name
    std::string group_name = "";
    for (boost::property_tree::ptree::value_type & group_child : group.second) {
      if (group_child.first == "<xmlattr>") {
        for (boost::property_tree::ptree::value_type & group_att : group_child.second) {
          if (group_att.first == "name") {
            group_name = group_att.second.data();
            RCLCPP_DEBUG_STREAM(logger_, "Group name: " << group_name);
          }
        }
      } else if (group_child.first == "joint") {
        for (boost::property_tree::ptree::value_type & joint_att :
          group_child.second.get_child("<xmlattr>"))
        {
          // Add only actuable joints, that are not present in the extra_joints list
          if (joint_att.first == "name" &&
            actuable_joints.find(joint_att.second.data()) != actuable_joints.end() &&
            extra_joints_.find(joint_att.second.data()) == extra_joints_.end())
          {
            addJointToGroup(group_name, joint_att.second.data());
            RCLCPP_DEBUG_STREAM(
              logger_,
              "Add joint " << joint_att.second.data() << " to " << group_name);
          }
        }
      } else if (group_child.first == "group") {
        for (boost::property_tree::ptree::value_type & subgroup_att :
          group_child.second.get_child("<xmlattr>"))
        {
          if (subgroup_att.first == "name") {
            if (!addGroupToGroup(group_name, subgroup_att.second.data())) {
              pending_groups_.push_back(std::make_pair(group_name, subgroup_att.second.data()));
              RCLCPP_DEBUG_STREAM(
                logger_,
                "Wait for group" << subgroup_att.second.data() << " to be loaded");
            }
            RCLCPP_DEBUG_STREAM(
              logger_,
              "Add group " << subgroup_att.second.data() << " to " << group_name);
          }
        }
      }
    }
  }
  // Process subgroups that weren't loaded in order
  for (const auto & pair : pending_groups_) {
    addGroupToGroup(pair.first, pair.second);
  }

  // Clean empty groups
  std::vector<std::string> groups_to_remove;
  for (const auto & pair : joint_groups_) {
    if (pair.second.size() == 0) {
      groups_to_remove.push_back(pair.first);
    }
  }
  for (const auto & group : groups_to_remove) {
    joint_groups_.erase(joint_groups_.find(group));
  }

  // Add empty group
  joint_groups_["None"] = {};
  RCLCPP_DEBUG_STREAM(logger_, "Joints loaded");
}

void Motion::setParamName()
{
  std::string random = "";
  static const char num[] = "0123456789";
  for (int i = 0; i < 5; ++i) {
    random.append(std::to_string(num[rand() % (sizeof(num) - 1)]));
  }

  tmp_name_ = "m_" + random;
}

void Motion::addKeyFrame(
  const sensor_msgs::msg::JointState & msg,
  float time_increment)
{
  KeyFrame k(logger_, time_increment);

  if (keyframes_.size() == 0) {   // The first frame has default time 0
    k.setTime(0.0);
  }

  for (unsigned int i = 0; i < msg.name.size(); ++i) {
    if (isJointUsed(msg.name[i])) {
      k.addPosition(msg.name[i], msg.position[i]);
    }
  }

  keyframes_.push_back(k);
}

KeyFrame & Motion::getKeyFrame(const int frame)
{
  if (static_cast<uint64_t>(frame) >= keyframes_.size()) {
    throw std::out_of_range("Keyframe " + std::to_string(frame) + " doesn't exist");
  }

  return keyframes_[frame];
}

void Motion::updateKeyFrame(const sensor_msgs::msg::JointState & msg, int frame)
{
  for (std::vector<JointPosition>::iterator it = keyframes_[frame].getJoints().begin();
    it != keyframes_[frame].getJoints().end(); ++it)
  {
    for (unsigned int i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == it->joint_name_) {
        it->position_ = msg.position[i];
        break;
      }
    }
  }
}

void Motion::addJointModel(const std::string & joint_name, const JointModel & joint_model)
{
  joint_models_[joint_name] = joint_model;
}

void Motion::changeTime(int frame, float time_increment)
{
  if (static_cast<uint64_t>(frame) >= keyframes_.size()) {
    RCLCPP_ERROR_STREAM(logger_, "Keyframe " << frame << " doesn't exist");
    throw std::invalid_argument("Keyframe " + std::to_string(frame) + " doesn't exist");
  }

  keyframes_[frame].setTime(time_increment);
}

double Motion::changeJoint(int frame, const std::string & joint_name, double position)
{
  if (static_cast<uint64_t>(frame) >= keyframes_.size()) {
    RCLCPP_ERROR_STREAM(logger_, "Keyframe " << frame << " doesn't exist");
    throw std::invalid_argument("Keyframe " + std::to_string(frame) + " doesn't exist");
  }

  for (std::vector<JointPosition>::iterator it = keyframes_[frame].getJoints().begin();
    it != keyframes_[frame].getJoints().end(); ++it)
  {
    if (it->joint_name_ == joint_name) {
      if (joint_models_[joint_name].inLimits(position)) {
        it->position_ = position;
      }

      return it->position_;   // Done
    }
  }

  RCLCPP_ERROR_STREAM(logger_, "Joint " << joint_name << " doesn't exist");
  throw std::invalid_argument("Joint " + joint_name + " doesn't exist");
  throw std::runtime_error("");
  return -1.0;
}

void Motion::removeKeyFrame(int frame)
{
  keyframes_.erase(keyframes_.begin() + frame);
}

void Motion::copyFrame(int frame, int new_frame_pos)
{
  KeyFrame k(keyframes_[frame]);

  const auto EPSILON = 1e-10;
  if (std::abs(k.getTime()) < EPSILON) {   // Only the first frame may have time 0
    k.setTime(DEFAULT_TIME);
  }

  if (new_frame_pos < 0) {
    keyframes_.push_back(k);
  } else {
    keyframes_.insert(keyframes_.begin() + new_frame_pos, k);
  }
}

void Motion::extendFrames(const sensor_msgs::msg::JointState & msg)
{
  for (unsigned int i = 0; i < msg.name.size(); ++i) {
    if (!isJointUsed(msg.name[i])) {
      for (std::vector<KeyFrame>::iterator it = keyframes_.begin(); it != keyframes_.end();
        ++it)
      {
        if (std::isnan(it->getJointPosition(msg.name[i]))) {
          it->addPosition(msg.name[i], msg.position[i]);
        }
      }
    }
  }
}

void Motion::addJointToGroup(const std::string & group, const std::string & joint)
{
  if (joint_groups_.find(group) == joint_groups_.end()) {
    joint_groups_[group] = {};
  }

  joint_groups_.at(group).push_back(joint);
}

bool Motion::addGroupToGroup(const std::string & group, const std::string & subgroup)
{
  if (joint_groups_.find(group) == joint_groups_.end()) {
    joint_groups_[group] = {};
  }

  if (joint_groups_.find(subgroup) != joint_groups_.end()) {
    joint_groups_.at(group).insert(
      joint_groups_.at(group).end(),
      joint_groups_.at(subgroup).begin(),
      joint_groups_.at(subgroup).end());
    return true;
  } else {
    // Subgroup is not yet loaded
    return false;
  }
}

PrintMotion Motion::print(double downshift) const
{
  return print("", "", "", downshift);
}

void Motion::removeAllKeyFrames()
{
  keyframes_.clear();
}

PrintMotion Motion::print(
  const std::string & name, const std::string & usage,
  const std::string & description, double downshift) const
{
  PrintMotion pm;
  pm.joints_ = getJoints();

  double basetime = 0.0;
  for (const auto & frame : keyframes_) {
    const auto point = frame.get_point(basetime, downshift, pm.joints_);
    pm.positions_.insert(pm.positions_.end(), point.positions_.begin(), point.positions_.end());
    pm.times_.push_back(point.time_from_start_);
    basetime = pm.times_.back();
  }

  if (name != "" || usage != "" || description != "") {
    pm.meta_.print_ = true;
    pm.meta_.name_ = name;
    pm.meta_.usage_ = usage;
    pm.meta_.description_ = description;
  } else {
    pm.meta_.print_ = false;
  }

  return pm;
}

std::string cleanName(const std::string & name)
{
  return name.substr(0, name.size() - 6);
}

std::string rosifyName(const std::string & name)
{
  return name + "_joint";
}
}  // namespace play_motion_builder

namespace YAML
{
Emitter & operator<<(YAML::Emitter & out, const play_motion_builder::PrintMotion & m)
{
  // Ensure precision is set for floating-point numbers
  out.SetFloatPrecision(5);
  out.SetDoublePrecision(5);

  // Lambda to format double values to 5 decimal places and always show .0 for whole numbers
  auto formatInteger = [](double value) -> std::string {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << value;
      return ss.str();
    };

  // Emit YAML with formatted doubles
  out << YAML::BeginMap
      << YAML::Key << play_motion_builder::PrintMotion::JOINTS_KEY
      << YAML::Value << YAML::Flow << m.joints_
      << YAML::Key << play_motion_builder::PrintMotion::POSITIONS_KEY
      << YAML::Value << YAML::Flow;

  out << YAML::BeginSeq;
  for (const auto & pos : m.positions_) {
    const int int_pos = static_cast<int>(pos);
    if (std::abs(pos) > std::abs(int_pos)) {  // Check if the number is an integer
      out << pos;
    } else {
      out << formatInteger(pos);
    }
  }
  out << YAML::EndSeq;

  out << YAML::Key << play_motion_builder::PrintMotion::TIME_KEY
      << YAML::Value << YAML::Flow;

  out << YAML::BeginSeq;
  for (const auto & time : m.times_) {
    const int int_time = static_cast<int>(time);
    if (std::abs(time) > std::abs(int_time)) {   // Check if the number is an integer
      out << time;
    } else {
      out << formatInteger(time);
    }
  }
  out << YAML::EndSeq;

  out << m.meta_
      << YAML::EndMap;

  return out;
}

Emitter & operator<<(YAML::Emitter & out, const play_motion_builder::PrintMeta & m)
{
  if (m.print_) {
    out << YAML::Key << play_motion_builder::PrintMeta::META_KEY << YAML::Value
        << YAML::BeginMap
        << YAML::Key << play_motion_builder::PrintMeta::NAME_KEY
        << YAML::Value << m.name_
        << YAML::Key << play_motion_builder::PrintMeta::USAGE_KEY
        << YAML::Value << YAML::Flow << m.usage_
        << YAML::Key << play_motion_builder::PrintMeta::DESC_KEY
        << YAML::Value << YAML::Flow << m.description_
        << YAML::EndMap;
  }

  return out;
}
}  // namespace YAML
