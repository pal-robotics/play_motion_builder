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

#ifndef PLAY_MOTION_BUILDER__MOTION_MODEL_HPP_
#define PLAY_MOTION_BUILDER__MOTION_MODEL_HPP_

#include <algorithm>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "play_motion2_msgs/msg/motion.hpp"

#include "rclcpp/logger.hpp"

#include "sensor_msgs/msg/joint_state.hpp"


namespace play_motion_builder
{
/** Helper structs for YAML printing **/
struct Point
{
  float time_from_start_;
  std::vector<double> positions_;
};
struct PrintMeta
{
  static const char META_KEY[];
  static const char NAME_KEY[];
  static const char USAGE_KEY[];
  static const char DESC_KEY[];

  bool print_;
  std::string name_;
  std::string usage_;
  std::string description_;
};
struct PrintMotion
{
  static const char JOINTS_KEY[];
  static const char POINTS_KEY[];
  static const char TIME_KEY[];
  static const char POSITIONS_KEY[];

  std::vector<std::string> joints_;
  std::vector<double> positions_;
  std::vector<float> times_;
  PrintMeta meta_;
};

/** Motion model **/

/**
* The JointGroup Struct represents a joint group as defined
* by move_it
*/
struct JointGroup
{
  std::string group_name_;
  std::vector<std::string> joints_;

  JointGroup()
  {
  }

  explicit JointGroup(const std::string & name)
  : group_name_(name)
  {
  }
};
/**
* The JointModel Struct represent the limits of a given joint
*/
struct JointModel
{
  double lower_limit_;
  double upper_limit_;

  JointModel()
  {
  }

  JointModel(double lower_limit, double upper_limit)
  : lower_limit_(lower_limit), upper_limit_(upper_limit)
  {
  }

  bool inLimits(double position)
  {
    return lower_limit_ <= position && upper_limit_ >= position;
  }
};
/**
* The JointPosition Struct represent the position of a given joint
*/
struct JointPosition
{
  std::string joint_name_;
  double position_;

  JointPosition(const std::string & joint_name, double position)
  : joint_name_(joint_name), position_(position)
  {
  }
};
/**
* The KeyFrame class represents a specific joint configuration at a given time
*/
class KeyFrame
{
public:
  KeyFrame(const rclcpp::Logger & logger, float time_increment);
  explicit KeyFrame(const KeyFrame & k);
  KeyFrame & operator=(const KeyFrame & other) = default;

  void addPosition(const std::string & name, double position);
  double getJointPosition(const std::string & joint) const;
  void cleanUnused(const std::map<std::string, bool> & used_joints);
  Point get_point(double basetime, double downshift, const std::vector<std::string> & names) const;
  PrintMotion print(const std::vector<std::string> & names) const;

  float getTime() const
  {
    return time_increment_;
  }
  void setTime(float time_increment)
  {
    time_increment_ = time_increment;
  }
  const std::vector<JointPosition> & getJoints() const
  {
    return joints_;
  }
  std::vector<JointPosition> & getJoints()
  {
    return joints_;
  }

private:
  rclcpp::Logger logger_;

  std::vector<JointPosition> joints_;
  float time_increment_;
};
/**
* The Motion class represents a complex motion of the robot, encoded as
* a sequence of Keyframes at specific times
*/
class Motion
{
public:
  static const float DEFAULT_TIME;

  Motion(
    const rclcpp::Logger & logger,
    const std::string & robot_description, const std::string & robot_description_semantic,
    const std::vector<std::string> & extra_joints);
  Motion(
    const rclcpp::Logger & logger,
    const play_motion2_msgs::msg::Motion & motion_cfg,
    const std::string & robot_description,
    const std::string & robot_description_semantic,
    const std::vector<std::string> & extra_joints);

  void setMotionGroups(
    const std::string & robot_description,
    const std::string & robot_description_semantic);
  void setParamName();
  void addKeyFrame(
    const sensor_msgs::msg::JointState & msg,
    float time_increment = DEFAULT_TIME);
  KeyFrame & getKeyFrame(const int frame);
  void updateKeyFrame(const sensor_msgs::msg::JointState & msg, int frame);
  void addJointModel(const std::string & joint_name, const JointModel & joint_model);
  void changeTime(int frame, float time_increment);
  double changeJoint(int frame, const std::string & joint_name, double position);
  void removeKeyFrame(int frame);
  void copyFrame(int frame, int new_frame_pos = -1);
  void loadFrame(int frame) const;
  void loadYAML(double downshift) const;
  void extendFrames(const sensor_msgs::msg::JointState & msg);
  PrintMotion print(double downshift = 1.0) const;
  PrintMotion print(
    const std::string & name, const std::string & usage,
    const std::string & description, double downshift = 1.0) const;

  void removeAllKeyFrames();
  void addJointToGroup(const std::string & group, const std::string & joint);
  bool addGroupToGroup(const std::string & group, const std::string & subgroup);

  const std::string & getParamName() const
  {
    return tmp_name_;
  }
  bool jointsLoaded()
  {
    return !joint_groups_.empty();
  }
  const KeyFrame & getLastKeyFrame() const
  {
    return getKeyFrame(keyframes_.size() - 1);
  }
  const KeyFrame & getKeyFrame(int i) const
  {
    return keyframes_[i];
  }
  std::vector<std::string> getJoints() const
  {
    std::vector<std::string> joints;

    // If using a joint group, list it
    if (group_used_ != "") {
      joints.insert(
        joints.end(), joint_groups_.at(group_used_).begin(),
        joint_groups_.at(group_used_).end());
    }

    // If using ungrouped joints, add them
    for (const auto & extra_joint : extra_joints_) {
      if (extra_joint.second) {
        joints.push_back(extra_joint.first);
      }
    }

    return joints;
  }
  std::size_t size() const
  {
    return keyframes_.size();
  }
  const std::vector<KeyFrame> getKeyframes() const
  {
    return keyframes_;
  }
  const std::string & getCurrentGroup()
  {
    return group_used_;
  }

  void addJointPositionToNanJointKeyframes(
    const sensor_msgs::msg::JointState & joint_state_msg,
    const std::string & joint_name)
  {
    if (joint_state_msg.name.empty()) {
      return;
    }
    // test if the name is found in joint_state msg
    auto joint_names_it =
      std::find(joint_state_msg.name.begin(), joint_state_msg.name.end(), joint_name);

    if (joint_names_it != joint_state_msg.name.end()) {
      // get index of joint_state data for name value
      unsigned int joint_state_name_index = joint_names_it - joint_state_msg.name.begin();

      for (auto & kf : keyframes_) {
        // If the joint is not yet on the list, add it, and set it's value to the current position
        if (std::find_if(
            kf.getJoints().begin(), kf.getJoints().end(),
            [&joint_name](JointPosition & jp) {
              return jp.joint_name_ == joint_name;
            }) == kf.getJoints().end())
        {
          kf.addPosition(joint_name, joint_state_msg.position[joint_state_name_index]);
        }
      }
    }
  }

  bool setCurrentGroup(
    const std::string & group,
    const sensor_msgs::msg::JointState & joint_state_msg)
  {
    if (joint_groups_.find(group) != joint_groups_.end()) {
      group_used_ = group;

      // verify if some active joints have nan values on current keyframes
      if (!joint_state_msg.name.empty()) {
        for (auto joint : getJoints()) {
          addJointPositionToNanJointKeyframes(joint_state_msg, joint);
        }
      }
      return true;
    } else {
      return false;
    }
  }
  bool isJointUsed(const std::string & joint_name) const
  {
    if (extra_joints_.find(joint_name) != extra_joints_.end()) {
      return extra_joints_.at(joint_name);
    } else if (group_used_ != "") {
      return std::find(
        joint_groups_.at(group_used_).begin(),
        joint_groups_.at(group_used_).end(),
        joint_name) != joint_groups_.at(group_used_).end();
    } else {
      return false;
    }
  }
  bool setExtraJointUsedState(
    const std::string & name, bool used,
    const sensor_msgs::msg::JointState & joint_state_msg = sensor_msgs::msg::JointState())
  {
    if (extra_joints_.find(name) != extra_joints_.end()) {
      extra_joints_[name] = used;

      // verify if some active joints have nan values on current keyframes
      if (!joint_state_msg.name.empty() && used) {
        addJointPositionToNanJointKeyframes(joint_state_msg, name);
      }

      return true;
    } else {
      return false;
    }
  }
  std::vector<std::string> getJointGroups()
  {
    std::vector<std::string> joint_groups;
    for (const auto & pair : joint_groups_) {
      joint_groups.push_back(pair.first);
    }

    return joint_groups;
  }
  std::vector<std::string> getExtraJoints()
  {
    std::vector<std::string> extra_joints;
    for (const auto & pair : extra_joints_) {
      extra_joints.push_back(pair.first);
    }

    return extra_joints;
  }
  std::vector<std::string> getAvailableJoints()
  {
    std::vector<std::string> joints;
    for (const auto & pair : joint_groups_) {
      for (const auto & joint : pair.second) {
        if (std::find(joints.begin(), joints.end(), joint) == joints.end()) {
          joints.push_back(joint);
        }
      }
    }

    // Add extra joints
    for (const auto & pair : extra_joints_) {
      joints.push_back(pair.first);
    }

    return joints;
  }

private:
  rclcpp::Logger logger_;

  std::string tmp_name_;
  std::vector<KeyFrame> keyframes_;
  std::unordered_map<std::string, std::vector<std::string>> joint_groups_;
  std::unordered_map<std::string, bool> extra_joints_;
  std::string group_used_;
  std::unordered_map<std::string, JointModel> joint_models_;
};

/** Helper functions for printing **/
std::string cleanName(const std::string & name);
std::string rosifyName(const std::string & name);
}  // namespace play_motion_builder

/** YAML Print functions **/
namespace YAML
{
Emitter & operator<<(YAML::Emitter & out, const play_motion_builder::PrintMotion & m);
Emitter & operator<<(YAML::Emitter & out, const play_motion_builder::PrintMeta & m);
}  // namespace YAML

#endif  // PLAY_MOTION_BUILDER__MOTION_MODEL_HPP_
