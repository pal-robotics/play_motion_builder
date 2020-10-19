#ifndef _H_PLAY_MOTION_BUILDER_MOTION_MODEL
#define _H_PLAY_MOTION_BUILDER_MOTION_MODEL

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <algorithm>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

namespace pal
{
/** Helper structs for YAML printing **/
struct PrintPoint
{
  const static std::string TIME_KEY;
  const static std::string POSITIONS_KEY;

  float time_from_start_;
  std::vector<double> positions_;
};
struct PrintMeta
{
  const static std::string META_KEY;
  const static std::string NAME_KEY;
  const static std::string USAGE_KEY;
  const static std::string DESC_KEY;

  bool print_;
  std::string name_;
  std::string usage_;
  std::string description_;
};
struct PrintMotion
{
  const static std::string JOINTS_KEY;
  const static std::string POINTS_KEY;

  std::vector<std::string> joints_;
  std::vector<PrintPoint> points_;
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

  JointGroup(const std::string& name) : group_name_(name)
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

  JointPosition(const std::string& joint_name, double position)
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
  KeyFrame(float time_increment);
  KeyFrame(const KeyFrame& k);

  void addPosition(const std::string& name, double position);
  double getJointPosition(const std::string& joint) const;
  void cleanUnused(const std::map<std::string, bool>& used_joints);
  PrintPoint print(double basetime, double downshift, const std::vector<std::string>& names) const;
  PrintMotion print(const std::vector<std::string>& names) const;

  float getTime() const
  {
    return time_increment_;
  }
  void setTime(float time_increment)
  {
    time_increment_ = time_increment;
  }
  const std::vector<JointPosition>& getJoints() const
  {
    return joints_;
  }
  std::vector<JointPosition>& getJoints()
  {
    return joints_;
  }

private:
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
  const static float DEFAULT_TIME;

  Motion(const std::string& robot_description, const std::string& robot_description_semantic,
         const std::vector<std::string>& extra_joints);
  Motion(XmlRpc::XmlRpcValue& param, const std::string& robot_description,
         const std::string& robot_description_semantic,
         const std::vector<std::string>& extra_joints);

  void setMotionGroups(const std::string& robot_description,
                       const std::string& robot_description_semantic);
  void setParamName();
  void addKeyFrame(const sensor_msgs::JointStateConstPtr& msg, float time_increment = DEFAULT_TIME);
  void updateKeyFrame(const sensor_msgs::JointStateConstPtr& msg, int frame);
  void addJointModel(const std::string& joint_name, const JointModel& joint_model);
  void changeTime(int frame, float time_increment);
  double changeJoint(int frame, const std::string& joint_name, double position);
  void removeKeyFrame(int frame);
  void copyFrame(int frame, int new_frame_pos = -1);
  void loadFrame(int frame) const;
  void loadYAML(double downshift) const;
  void extendFrames(const sensor_msgs::JointStateConstPtr& msg);
  PrintMotion print(double downshift = 1.0) const;
  PrintMotion print(const std::string& name, const std::string& usage,
                    const std::string& description, double downshift = 1.0) const;

  void removeAllKeyFrames();
  void addJointToGroup(const std::string& group, const std::string& joint);
  void addGroupToGroup(const std::string& group, const std::string& subgroup);

  const std::string& getParamName() const
  {
    return tmp_name_;
  }
  bool jointsLoaded()
  {
    return !joint_groups_.empty();
  }
  const KeyFrame& getLastKeyFrame() const
  {
    return getKeyFrame(keyframes_.size() - 1);
  }
  const KeyFrame& getKeyFrame(int i) const
  {
    return keyframes_[i];
  }
  std::vector<std::string> getJoints() const
  {
    std::vector<std::string> joints;

    // If using a joint group, list it
    if (group_used_ != "")
    {
      joints.insert(joints.end(), joint_groups_.at(group_used_).begin(),
                    joint_groups_.at(group_used_).end());
    }

    // If using ungrouped joints, add them
    for (const auto& extra_joint : extra_joints_)
    {
      if (extra_joint.second)
        joints.push_back(extra_joint.first);
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
  const std::string& getCurrentGroup()
  {
    return group_used_;
  }
  bool setCurrentGroup(const std::string& group)
  {
    if (joint_groups_.find(group) != joint_groups_.end())
    {
      group_used_ = group;
      return true;
    }
    else
    {
      return false;
    }
  }
  bool isJointUsed(const std::string& joint_name) const
  {
    if (extra_joints_.find(joint_name) != extra_joints_.end())
      return extra_joints_.at(joint_name);
    else if (group_used_ != "")
      return std::find(joint_groups_.at(group_used_).begin(),
                       joint_groups_.at(group_used_).end(),
                       joint_name) != joint_groups_.at(group_used_).end();
    else
      return false;
  }
  bool setExtraJointUsedState(const std::string& name, bool used)
  {
    if (extra_joints_.find(name) != extra_joints_.end())
    {
      extra_joints_[name] = used;
      return true;
    }
    else
    {
      return false;
    }
  }
  std::vector<std::string> getJointGroups()
  {
    std::vector<std::string> joint_groups;
    for (const auto& pair : joint_groups_)
    {
      joint_groups.push_back(pair.first);
    }

    return joint_groups;
  }
  std::vector<std::string> getExtraJoints()
  {
    std::vector<std::string> extra_joints;
    for (const auto& pair : extra_joints_)
    {
      extra_joints.push_back(pair.first);
    }

    return extra_joints;
  }
  std::vector<std::string> getAvailableJoints()
  {
    std::vector<std::string> joints;
    for (const auto& pair : joint_groups_)
    {
      for (const auto& joint : pair.second)
      {
        if (std::find(joints.begin(), joints.end(), joint) == joints.end())
        {
          joints.push_back(joint);
        }
      }
    }

    // Add extra joints
    for (const auto& pair : extra_joints_)
    {
      joints.push_back(pair.first);
    }

    return joints;
  }

private:
  std::string tmp_name_;
  std::vector<KeyFrame> keyframes_;
  std::unordered_map<std::string, std::vector<std::string> > joint_groups_;
  std::unordered_map<std::string, bool> extra_joints_;
  std::string group_used_;
  std::unordered_map<std::string, JointModel> joint_models_;
};

/** Helper functions for printing **/
void loadParams(const YAML::Emitter& param, const std::string& filename);
std::string cleanName(const std::string& name);
std::string rosifyName(const std::string& name);
double toDouble(XmlRpc::XmlRpcValue& value);
}  // namespace pal

/** YAML Print functions **/
namespace YAML
{
Emitter& operator<<(YAML::Emitter& out, const pal::PrintMotion& m);
Emitter& operator<<(YAML::Emitter& out, const pal::PrintPoint& k);
Emitter& operator<<(YAML::Emitter& out, const pal::PrintMeta& m);
}  // namespace YAML

#endif /* _H_PLAY_MOTION_BUILDER_MOTION_MODEL */
