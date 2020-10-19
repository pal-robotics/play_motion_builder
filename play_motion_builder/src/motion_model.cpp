#include <play_motion_builder/motion_model.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <fstream>
#include <stdio.h>
#include <cstdio>
#include <unordered_set>

namespace pal
{
const std::string PrintPoint::TIME_KEY = "time_from_start";
const std::string PrintPoint::POSITIONS_KEY = "positions";
const std::string PrintMotion::JOINTS_KEY = "joints";
const std::string PrintMotion::POINTS_KEY = "points";
const std::string PrintMeta::META_KEY = "meta";
const std::string PrintMeta::NAME_KEY = "name";
const std::string PrintMeta::USAGE_KEY = "usage";
const std::string PrintMeta::DESC_KEY = "description";

namespace
{
std::string exec(const char *cmd)
{
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
    return "Error";
  char buffer[1024];
  std::string result = "";
  while (!feof(pipe.get()))
  {
    if (fgets(buffer, 128, pipe.get()) != NULL)
      result += buffer;
  }
  return result;
}
}  // namespace

KeyFrame::KeyFrame(float time_increment) : time_increment_(time_increment)
{
}

KeyFrame::KeyFrame(const KeyFrame &k) : time_increment_(k.time_increment_)
{
  joints_ = k.joints_;
}

void KeyFrame::addPosition(const std::string &name, double position)
{
  joints_.push_back(JointPosition(name, position));
}

double KeyFrame::getJointPosition(const std::string &joint) const
{
  for (auto joint_position : joints_)
  {
    if (joint_position.joint_name_ == joint)
    {
      return joint_position.position_;
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

void KeyFrame::cleanUnused(const std::map<std::string, bool> &used_joints)
{
  for (unsigned int i = joints_.size(); i > 0; --i)  // Iterate backwards and remove unused
  {
    try
    {
      if (!used_joints.at(joints_[i - 1].joint_name_))
      {
        joints_.erase(joints_.begin() + i - 1);
      }
    }
    catch (std::out_of_range &e)
    {
      ROS_ERROR_STREAM("Key: " << joints_[i - 1].joint_name_ << " is unknown");
      throw e;
    }
  }
}

PrintPoint KeyFrame::print(double basetime, double downshift,
                           const std::vector<std::string> &names) const
{
  PrintPoint pp;
  pp.time_from_start_ = basetime + (downshift * time_increment_);

  for (auto joint_name : names)
  {
    pp.positions_.push_back(getJointPosition(joint_name));
  }

  return pp;
}

PrintMotion KeyFrame::print(const std::vector<std::string> &names) const
{
  PrintMotion pm;
  PrintPoint pp;

  for (auto joint : joints_)
  {
    if (std::find(names.begin(), names.end(), joint.joint_name_) != names.end())
    {
      pm.joints_.push_back(joint.joint_name_);
      pp.positions_.push_back(joint.position_);
    }
  }

  pp.time_from_start_ = 0.0;
  pm.points_.push_back(pp);
  pm.meta_.print_ = false;
  return pm;
}

const float Motion::DEFAULT_TIME = 5.0;

Motion::Motion(const std::string &robot_description, const std::string &robot_description_semantic,
               const std::vector<std::string> &extra_joints)
  : tmp_name_("")
{
  // Load groups
  if (robot_description != "" && robot_description_semantic != "")
  {
    setMotionGroups(robot_description, robot_description_semantic);

    // Pre-select biggest group
    uint size = 0;
    for (const auto &group : joint_groups_)
    {
      if (group.second.size() >= size)
      {
        group_used_ = group.first;
        size = group.second.size();
      }
    }
  }

  // Collect extra joints
  for (const auto &joint : extra_joints)
  {
    extra_joints_[joint] = true;
  }
}

Motion::Motion(XmlRpc::XmlRpcValue &param, const std::string &robot_description,
               const std::string &robot_description_semantic,
               const std::vector<std::string> &extra_joints)
  : Motion(robot_description, robot_description_semantic, extra_joints)
{
  if (param["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // Load used joints
    ROS_INFO("Joint used:");
    std::vector<std::string> names;
    for (int i = 0; i < param["joints"].size(); ++i)
    {
      names.push_back(static_cast<std::string>(param["joints"][i]));
      ROS_INFO_STREAM(static_cast<std::string>(param["joints"][i]));
    }

    // Set state for extra joints
    for (auto &joint : extra_joints_)
    {
      joint.second = std::find(names.begin(), names.end(), joint.first) != names.end();
    }

    // Find used group (biggest group with all joints used)
    std::string group_used = "";
    uint size = 0;
    for (const auto &group : joint_groups_)
    {
      // Used group can't have more joints, only check if bigger than found
      if (group.second.size() <= names.size() && group.second.size() > size)
      {
        // Check if all joints are used
        bool all_used = true;
        for (const std::string &joint : group.second)
        {
          if (std::find(names.begin(), names.end(), joint) == names.end())
          {
            ROS_INFO_STREAM("Joint " << joint << " from group " << group.first << " not in use");
            all_used = false;
            break;
          }
        }

        // If all used, choose this one
        if (all_used)
        {
          size = group.second.size();
          group_used = group.first;
          ROS_INFO_STREAM("Found group candidate " << group_used);
        }
      }
      else
      {
        ROS_INFO_STREAM("Group " << group.first << " discarded (" << group.second.size()
                                 << "<=" << names.size() << " && " << group.second.size()
                                 << ">" << size << ")");
      }
    }
    group_used_ = group_used;

    // Populate motion
    if (param["points"].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      float last_time = 0.0f;
      for (int i = 0; i < param["points"].size(); ++i)
      {
        KeyFrame k(toDouble(param["points"][i]["time_from_start"]) - last_time);
        // Load joints in order
        for (int j = 0; j < param["points"][i]["positions"].size(); ++j)
        {
          k.addPosition(names[j], toDouble(param["points"][i]["positions"][j]));
        }
        keyframes_.push_back(k);
        last_time += k.getTime();
      }
    }
  }
}

void Motion::setMotionGroups(const std::string &robot_description,
                             const std::string &robot_description_semantic)
{
  // Load XML robot's description
  boost::property_tree::ptree tree_robot;
  std::stringstream ss_robot(robot_description);
  boost::property_tree::read_xml(ss_robot, tree_robot);

  // Find all actuable joints
  std::unordered_set<std::string> actuable_joints;
  for (boost::property_tree::ptree::value_type &joint : tree_robot.get_child("robot"))
  {
    if (joint.first != "joint")  // Ignore others
      continue;

    // Get joint name
    std::string joint_name;
    bool is_actuable = true;
    for (boost::property_tree::ptree::value_type &joint_child : joint.second)
    {
      if (joint_child.first == "<xmlattr>")
      {
        for (boost::property_tree::ptree::value_type &joint_att : joint_child.second)
        {
          if (joint_att.first == "name")
          {
            joint_name = joint_att.second.data();
          }
          else if (joint_att.first == "type" && joint_att.second.data() == "fixed")
            is_actuable = false;
        }
      }
    }

    if (is_actuable)
    {
      actuable_joints.insert(joint_name);
      ROS_DEBUG_STREAM("Actuable joint: " << joint_name);
    }
  }

  // Load XML groups description
  boost::property_tree::ptree tree_semantic;
  std::stringstream ss_semantic(robot_description_semantic);
  boost::property_tree::read_xml(ss_semantic, tree_semantic);

  // Load groups
  for (boost::property_tree::ptree::value_type &group : tree_semantic.get_child("robot"))
  {
    if (group.first != "group")  // Ignore others
      continue;

    // Get group name
    std::string group_name = "";
    for (boost::property_tree::ptree::value_type &group_child : group.second)
    {
      if (group_child.first == "<xmlattr>")
      {
        for (boost::property_tree::ptree::value_type &group_att : group_child.second)
        {
          if (group_att.first == "name")
          {
            group_name = group_att.second.data();
            ROS_DEBUG_STREAM("Group name: " << group_name);
          }
        }
      }
      else if (group_child.first == "joint")
      {
        for (boost::property_tree::ptree::value_type &joint_att :
             group_child.second.get_child("<xmlattr>"))
        {
          // Add only actuable joints
          if (joint_att.first == "name" &&
              actuable_joints.find(joint_att.second.data()) != actuable_joints.end())
          {
            addJointToGroup(group_name, joint_att.second.data());
            ROS_DEBUG_STREAM("Add joint " << joint_att.second.data() << " to " << group_name);
          }
        }
      }
      else if (group_child.first == "group")
      {
        for (boost::property_tree::ptree::value_type &subgroup_att :
             group_child.second.get_child("<xmlattr>"))
        {
          if (subgroup_att.first == "name")
          {
            addGroupToGroup(group_name, subgroup_att.second.data());
            ROS_DEBUG_STREAM("Add group " << subgroup_att.second.data() << " to " << group_name);
          }
        }
      }
    }
  }
  // Add empty group
  joint_groups_["None"] = {};
}

void Motion::setParamName()
{
  std::string random = "";
  static const char num[] = "0123456789";
  for (int i = 0; i < 5; ++i)
  {
    random.append(std::to_string(num[rand() % (sizeof(num) - 1)]));
  }

  tmp_name_ = "m_" + random;
}

void Motion::addKeyFrame(const sensor_msgs::JointStateConstPtr &msg, float time_increment)
{
  KeyFrame k(time_increment);

  if (keyframes_.size() == 0)  // The first frame has default time 0
  {
    k.setTime(0.0);
  }

  for (unsigned int i = 0; i < msg->name.size(); ++i)
  {
    if (isJointUsed(msg->name[i]))
    {
      k.addPosition(msg->name[i], msg->position[i]);
    }
  }

  keyframes_.push_back(k);
}

void Motion::updateKeyFrame(const sensor_msgs::JointStateConstPtr &msg, int frame)
{
  for (std::vector<JointPosition>::iterator it = keyframes_[frame].getJoints().begin();
       it != keyframes_[frame].getJoints().end(); ++it)
  {
    for (unsigned int i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == it->joint_name_)
      {
        it->position_ = msg->position[i];
        break;
      }
    }
  }
}

void Motion::addJointModel(const std::string &joint_name, const JointModel &joint_model)
{
  joint_models_[joint_name] = joint_model;
}

void Motion::changeTime(int frame, float time_increment)
{
  if ((unsigned long)frame >= keyframes_.size())
  {
    ROS_ERROR_STREAM("Keyframe " << frame << " doesn't exist");
    throw ros::Exception("Keyframe " + std::to_string(frame) + " doesn't exist");
  }

  keyframes_[frame].setTime(time_increment);
}

double Motion::changeJoint(int frame, const std::string &joint_name, double position)
{
  if ((unsigned long)frame >= keyframes_.size())
  {
    ROS_ERROR_STREAM("Keyframe " << frame << " doesn't exist");
    throw ros::Exception("Keyframe " + std::to_string(frame) + " doesn't exist");
  }

  for (std::vector<JointPosition>::iterator it = keyframes_[frame].getJoints().begin();
       it != keyframes_[frame].getJoints().end(); ++it)
  {
    if (it->joint_name_ == joint_name)
    {
      if (joint_models_[joint_name].inLimits(position))
      {
        it->position_ = position;
      }

      return it->position_;  // Done
    }
  }

  ROS_ERROR_STREAM("Joint " << joint_name << " doesn't exist");
  throw ros::Exception("Joint " + joint_name + " doesn't exist");
}

void Motion::removeKeyFrame(int frame)
{
  keyframes_.erase(keyframes_.begin() + frame);
}

void Motion::copyFrame(int frame, int new_frame_pos)
{
  KeyFrame k(keyframes_[frame]);

  if (k.getTime() == 0)  // Only the first frame may have time 0
  {
    k.setTime(DEFAULT_TIME);
  }

  if (new_frame_pos < 0)
  {
    keyframes_.push_back(k);
  }
  else
  {
    keyframes_.insert(keyframes_.begin() + new_frame_pos, k);
  }
}

void Motion::loadFrame(int frame) const
{
  YAML::Emitter em;
  /* clang-format off */
  em << YAML::BeginMap
    << YAML::Key << "play_motion"
      << YAML::Value << YAML::BeginMap
        << YAML::Key << "motions"
            << YAML::Value <<  YAML::BeginMap
        << YAML::Key << getParamName()
            << YAML::Value << keyframes_[frame].print(getJoints())
      << YAML::EndMap
    << YAML::EndMap
  << YAML::EndMap;
  /* clang-format on */

  // Build file
  // ROS_INFO_STREAM("YAML: \n" << motion_yaml);
  loadParams(em, getParamName());
}

void Motion::loadYAML(double downshift) const
{
  YAML::Emitter em;
  /* clang-format off */
  em << YAML::BeginMap
    << YAML::Key << "play_motion"
      << YAML::Value << YAML::BeginMap
        << YAML::Key << "motions"
            << YAML::Value <<  YAML::BeginMap
        << YAML::Key << getParamName()
            << YAML::Value << print(downshift)
      << YAML::EndMap
    << YAML::EndMap
  << YAML::EndMap;
  /* clang-format on */

  // Build file
  // ROS_INFO_STREAM("YAML: \n" << motion_yaml);
  loadParams(em, getParamName());
}

void Motion::extendFrames(const sensor_msgs::JointStateConstPtr &msg)
{
  for (unsigned int i = 0; i < msg->name.size(); ++i)
  {
    if (!isJointUsed(msg->name[i]))
    {
      for (std::vector<KeyFrame>::iterator it = keyframes_.begin(); it != keyframes_.end(); ++it)
      {
        if (std::isnan(it->getJointPosition(msg->name[i])))
        {
          it->addPosition(msg->name[i], msg->position[i]);
        }
      }
    }
  }
}

void Motion::addJointToGroup(const std::string &group, const std::string &joint)
{
  if (joint_groups_.find(group) == joint_groups_.end())
    joint_groups_[group] = {};

  joint_groups_.at(group).push_back(joint);
}

void Motion::addGroupToGroup(const std::string &group, const std::string &subgroup)
{
  if (joint_groups_.find(group) == joint_groups_.end())
    joint_groups_[group] = {};

  joint_groups_.at(group).insert(joint_groups_.at(group).end(),
                                 joint_groups_.at(subgroup).begin(),
                                 joint_groups_.at(subgroup).end());
}

PrintMotion Motion::print(double downshift) const
{
  return print("", "", "", downshift);
}

void Motion::removeAllKeyFrames()
{
  keyframes_.clear();
}

PrintMotion Motion::print(const std::string &name, const std::string &usage,
                          const std::string &description, double downshift) const
{
  PrintMotion pm;
  pm.joints_ = getJoints();

  double basetime = 0.0;
  for (auto frame : keyframes_)
  {
    pm.points_.push_back(frame.print(basetime, downshift, pm.joints_));
    basetime = pm.points_.back().time_from_start_;
  }

  if (name != "" || usage != "" || description != "")
  {
    pm.meta_.print_ = true;
    pm.meta_.name_ = name;
    pm.meta_.usage_ = usage;
    pm.meta_.description_ = description;
  }
  else
  {
    pm.meta_.print_ = false;
  }

  return pm;
}

void loadParams(const YAML::Emitter &param, const std::string &name)
{
  // Clean param
  exec(("rosparam delete /play_motion/motions/" + name).c_str());

  // Open file
  std::ofstream ofile("/tmp/" + name + ".yaml");
  ofile << param.c_str();
  ROS_INFO_STREAM("File /tmp/" << name << ".yaml written");
  ofile.close();

  // Loat to ros
  exec(("rosparam load /tmp/" + name + ".yaml").c_str());
  ros::Duration d(1.0);
  d.sleep();
}

std::string cleanName(const std::string &name)
{
  return name.substr(0, name.size() - 6);
}

std::string rosifyName(const std::string &name)
{
  return name + "_joint";
}

double toDouble(XmlRpc::XmlRpcValue &value)
{
  double val = std::numeric_limits<double>::quiet_NaN();
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    val = static_cast<double>(value);
  }
  else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    val = static_cast<int>(value);
  }
  else
  {
    ROS_ERROR_STREAM("Unknown time type: " << value.getType());
  }

  return val;
}
}  // namespace pal

namespace YAML
{
Emitter &operator<<(YAML::Emitter &out, const pal::PrintMotion &m)
{
  out.SetFloatPrecision(5);
  out.SetDoublePrecision(5);


  /* clang-format off */
    out << YAML::BeginMap
      << YAML::Key << pal::PrintMotion::JOINTS_KEY
        << YAML::Value << YAML::Flow << m.joints_
      << YAML::Key << pal::PrintMotion::POINTS_KEY
         << m.points_
      << m.meta_
    << YAML::EndMap;
  /* clang-format on */

  return out;
}

Emitter &operator<<(YAML::Emitter &out, const pal::PrintPoint &k)
{
  out.SetFloatPrecision(5);
  out.SetDoublePrecision(5);

  /* clang-format off */
    out << YAML::BeginMap
      << YAML::Key << pal::PrintPoint::TIME_KEY
        << YAML::Value << k.time_from_start_
      << YAML::Key << pal::PrintPoint::POSITIONS_KEY
        << YAML::Value << YAML::Flow << k.positions_
    << YAML::EndMap;
  /* clang-format on */

  return out;
}

Emitter &operator<<(YAML::Emitter &out, const pal::PrintMeta &m)
{
  if (m.print_)
  {
    /* clang-format off */
      out << YAML::Key << pal::PrintMeta::META_KEY << YAML::Value
      << YAML::BeginMap
        << YAML::Key << pal::PrintMeta::NAME_KEY
          << YAML::Value << m.name_
        << YAML::Key << pal::PrintMeta::USAGE_KEY
          << YAML::Value << YAML::Flow << m.usage_
        << YAML::Key << pal::PrintMeta::DESC_KEY
          << YAML::Value << YAML::Flow << m.description_
      << YAML::EndMap;
    /* clang-format on */
  }

  return out;
}
}  // namespace YAML
