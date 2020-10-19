#include "play_motion_builder.h"

#include <sensor_msgs/JointState.h>
#include <fstream>

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

namespace pal
{
ROSMotionBuilderNode::ROSMotionBuilderNode()
  : private_nh_("~")
  , action_server_(private_nh_, "build", false)
  , run_motion_server_(private_nh_, "run",
                       boost::bind(&ROSMotionBuilderNode::executeRunMotionCb, this, _1), false)
  , play_motion_client_("/play_motion")
  , running_(false)
{
  // Register callbacks
  action_server_.registerGoalCallback(boost::bind(&ROSMotionBuilderNode::buildMotionGoalCb, this));
  action_server_.registerPreemptCallback(
      boost::bind(&ROSMotionBuilderNode::buildMotionPreemptCb, this));
  run_motion_server_.registerPreemptCallback(
      boost::bind(&ROSMotionBuilderNode::runMotionPreemptCb, this));
}

bool ROSMotionBuilderNode::initialize()
{
  // Get Robot descriptions
  std::string robot_desc_param =
      private_nh_.param("robot_description_param", std::string("/robot_description"));
  if (!nh_.getParam(robot_desc_param, robot_description_))
  {
    ROS_ERROR("Couldn't find robot description for this robot");
    return false;
  }
  std::string semantic_desc_param =
      private_nh_.param("semantic_description", std::string("/robot_description_semantic"));
  if (!nh_.getParam(semantic_desc_param, semantic_description_))
  {
    ROS_ERROR("Couldn't find semantic description for this robot");
    return false;
  }

  // Get extra joints
  std::string extra_joints_param_name = private_nh_.param(
      "extra_joints_param_name",
      std::string("/play_motion/approach_planner/exclude_from_planning_joints"));
  XmlRpc::XmlRpcValue extra_joints_param;
  if (nh_.getParam(extra_joints_param_name, extra_joints_param))
  {
    for (int i = 0; i < extra_joints_param.size(); ++i)
    {
      extra_joints_.push_back(static_cast<std::string>(extra_joints_param[i]));
    }
  }

  // Set up Services
  edit_motion_server_ =
      private_nh_.advertiseService("edit_motion", &ROSMotionBuilderNode::editMotionCb, this);
  store_motion_server_ =
      private_nh_.advertiseService("store_motion", &ROSMotionBuilderNode::storeMotionCb, this);
  change_joints_server_ =
      private_nh_.advertiseService("change_joints", &ROSMotionBuilderNode::changeJointsCb, this);
  list_joints_server_ = private_nh_.advertiseService(
      "list_joint_groups", &ROSMotionBuilderNode::listJointGroupsCb, this);

  // Start the action server
  action_server_.start();
  run_motion_server_.start();

  return true;
}

void ROSMotionBuilderNode::buildMotionGoalCb()
{
  play_motion_builder_msgs::BuildMotionGoalConstPtr goal = action_server_.acceptNewGoal();
  if (goal->motion != "")
  {
    // Load an existing motion
    XmlRpc::XmlRpcValue motion_cfg;
    if (nh_.getParam("/play_motion/motions/" + goal->motion, motion_cfg))
    {
      // Process and load motion
      motion_.reset(new Motion(motion_cfg, robot_description_, semantic_description_,
                               extra_joints_));
      ROS_INFO_STREAM("Motion " << goal->motion << " loaded");
      ROS_DEBUG_STREAM("Using joint group: " << motion_->getCurrentGroup());
    }
    else
    {
      play_motion_builder_msgs::BuildMotionResult result;
      result.ok = false;
      result.message = "Couldn't find motion " + goal->motion;
      action_server_.setAborted(result);
    }
  }
  else
  {
    // Create a new empty motion
    motion_.reset(new Motion(robot_description_, semantic_description_, extra_joints_));
    ROS_INFO("New motion ready");
  }

  // Ready to process commands
  running_ = true;
  ROS_INFO("Motion Builder Ready");
}

void ROSMotionBuilderNode::buildMotionPreemptCb()
{
  // Prevent any further actions on the motion
  running_ = false;

  // Preempt the action
  play_motion_builder_msgs::BuildMotionResult result;
  result.ok = false;
  result.message = "Process was preempted";
  action_server_.setPreempted(result);

  // Clear the motion
  motion_.reset();
}

void ROSMotionBuilderNode::runMotionPreemptCb()
{
  play_motion_client_.cancelAllGoals();
}

bool ROSMotionBuilderNode::editMotionCb(play_motion_builder_msgs::EditMotion::Request &req,
                                        play_motion_builder_msgs::EditMotion::Response &res)
{
  if (running_)
  {
    switch (req.action)
    {
      case play_motion_builder_msgs::EditMotion::Request::LIST:
        // Don't modify the motion
        break;
      case play_motion_builder_msgs::EditMotion::Request::APPEND:
      case play_motion_builder_msgs::EditMotion::Request::EDIT:
      {
        sensor_msgs::JointStateConstPtr joint_state =
            ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        if (req.action == play_motion_builder_msgs::EditMotion::Request::APPEND)
        {
          // Append keyframe at the end
          motion_->addKeyFrame(joint_state);
        }
        else
        {
          // Update requested frame id with current position
          motion_->updateKeyFrame(joint_state, req.step_id);
        }
      }
      break;
      case play_motion_builder_msgs::EditMotion::Request::COPY_AS_LAST:
      case play_motion_builder_msgs::EditMotion::Request::COPY_AS_NEXT:
        motion_->copyFrame(req.step_id,
                           req.action == play_motion_builder_msgs::EditMotion::Request::COPY_AS_LAST ?
                               -1 :
                               req.step_id + 1);
        break;
      case play_motion_builder_msgs::EditMotion::Request::REMOVE:
        motion_->removeKeyFrame(req.step_id);
        break;
      case play_motion_builder_msgs::EditMotion::Request::EDIT_TIME:
        motion_->changeTime(req.step_id, req.time);
        break;
      default:
        ROS_ERROR_STREAM("Unknown action code: " << req.action);
        res.ok = false;
        res.message = "Unknown action code";
    }

    // Return current motion
    motionToROSMsg(res.motion);
    res.ok = true;
  }
  else
  {
    res.ok = false;
    res.message = "No motion being built";
  }

  return true;
}

bool ROSMotionBuilderNode::storeMotionCb(play_motion_builder_msgs::StoreMotion::Request &req,
                                         play_motion_builder_msgs::StoreMotion::Response &res)
{
  if (running_)
  {
    /* clang-format off */
    YAML::Emitter e;
    e << YAML::BeginMap
        << YAML::Key << "play_motion"
        << YAML::Value << YAML::BeginMap
          << YAML::Key << "motions"
          << YAML::Value << YAML::BeginMap
            << YAML::Key << req.ros_name
            << YAML::Value << motion_->print(req.meta.name, req.meta.usage, req.meta.description)
          << YAML::EndMap
        << YAML::EndMap
      << YAML::EndMap;
    /* clang-format on */
    ROS_INFO_STREAM("Motion: " << e.c_str());

    // Write file at output path
    if (req.file_path != "")
    {
      try
      {
        std::ofstream file;
        file.open(req.file_path);
        file << e.c_str();
        file.close();

        // Clean param
        exec(("rosparam delete /play_motion/motions/" + req.ros_name).c_str());

        // Load to ros
        exec(("rosparam load " + req.file_path).c_str());
        ros::Duration d(1.0);
        d.sleep();
      }
      catch (const std::ios_base::failure &err)
      {
        res.ok = false;
        res.message = err.what();
        return true;
      }
    }
    
    res.ok = true;
    res.message = std::string(e.c_str());
  }
  else
  {
    res.ok = false;
    res.message = "No motion being built, could not store.";
  }

  return true;
}

bool ROSMotionBuilderNode::changeJointsCb(play_motion_builder_msgs::ChangeJoints::Request &req,
                                          play_motion_builder_msgs::ChangeJoints::Response &res)
{
  if (running_)
  {
    res.ok = true;

    // Change used group
    if (req.group != "")
    {
      if (!motion_->setCurrentGroup(req.group))
      {
        res.ok = false;
        res.message = "Couldn't change group to " + req.group + ". ";
      }
    }
    // Set joints to remove to Unused
    if (req.joints_to_remove.size() > 0)
    {
      for (const auto &joint : req.joints_to_remove)
      {
        // Set joint to unused
        if (!motion_->setExtraJointUsedState(joint, false))
        {
          res.ok = false;
          res.message += "Couldn't remove extra joint " + joint + ". ";
        }
      }
    }
    // Set joints_to_add to Used
    if (req.joints_to_add.size() > 0)
    {
      for (const auto &joint : req.joints_to_add)
      {
        // Set joint to used
        if (!motion_->setExtraJointUsedState(joint, true))
        {
          res.ok = false;
          res.message += "Couldn't add extra joint " + joint + ". ";
        }
      }
    }

    // Reply
    res.current_group = motion_->getCurrentGroup();
    res.used_joints = motion_->getJoints();
  }
  else
  {
    res.ok = false;
    res.message = "No motion being built";
  }

  return true;
}

bool ROSMotionBuilderNode::listJointGroupsCb(play_motion_builder_msgs::ListJointGroups::Request &,
                                             play_motion_builder_msgs::ListJointGroups::Response &res)
{
  if (running_)
  {
    res.groups = motion_->getJointGroups();
    res.additional_joints = motion_->getExtraJoints();
    res.available_joints = motion_->getAvailableJoints();
  }

  return true;
}

void ROSMotionBuilderNode::executeRunMotionCb(const play_motion_builder_msgs::RunMotionGoalConstPtr &goal)
{
  if (!running_)
  {
    run_motion_server_.setAborted();
  }
  else
  {
    // Set up param name
    if (motion_->getParamName() == "")
      motion_->setParamName();

    if (goal->run_mode == play_motion_builder_msgs::RunMotionGoal::RUN_MOTION)
    {
      // Load motion to rosparam
      motion_->loadYAML(goal->downshift == 0.0 ? 1.0 : goal->downshift);
    }
    else if (goal->run_mode == play_motion_builder_msgs::RunMotionGoal::GO_TO_STEP)
    {
      motion_->loadFrame(goal->step_id);
    }
    else
    {
      ROS_ERROR_STREAM("Unknown run mode: " << goal->run_mode);
      run_motion_server_.setPreempted();
    }

    // Execute the motion
    play_motion_msgs::PlayMotionGoal motion_goal;
    motion_goal.motion_name = motion_->getParamName();
    play_motion_client_.sendGoal(motion_goal);

    // Track goal
    while (!play_motion_client_.getState().isDone() && !run_motion_server_.isPreemptRequested())
    {
      ros::Duration(0.1).sleep();
    }

    // Motion finished with play motion state
    if (play_motion_client_.getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
      run_motion_server_.setSucceeded();
    else if (play_motion_client_.getState().state_ == actionlib::SimpleClientGoalState::ABORTED)
      run_motion_server_.setAborted(play_motion_builder_msgs::RunMotionResult(),
                                    play_motion_client_.getState().text_);
    else
      run_motion_server_.setPreempted();
  }
}

void ROSMotionBuilderNode::motionToROSMsg(play_motion_builder_msgs::Motion &motion)
{
  motion.joints = motion_->getJoints();
  motion.used_group = motion_->getCurrentGroup();
  for (unsigned int i = 0; i < motion_->size(); ++i)
  {
    play_motion_builder_msgs::Frame f;
    for (const auto &joint : motion.joints)
    {
      f.pose.push_back(motion_->getKeyFrame(i).getJointPosition(joint));
    }
    f.time_from_last = motion_->getKeyFrame(i).getTime();

    motion.keyframes.push_back(f);
  }
}

}  // namespace pal
