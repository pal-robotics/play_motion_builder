#ifndef H_ROS_MOTION_BUILDER
#define H_ROS_MOTION_BUILDER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

#include <ros_motion_builder_msgs/BuildMotionAction.h>
#include <ros_motion_builder_msgs/RunMotionAction.h>
#include <ros_motion_builder_msgs/EditMotion.h>
#include <ros_motion_builder_msgs/StoreMotion.h>
#include <ros_motion_builder_msgs/ChangeJoints.h>
#include <ros_motion_builder_msgs/ListJointGroups.h>

#include <ros_motion_builder/motion_model.h>

namespace pal
{
class ROSMotionBuilderNode
{
  typedef actionlib::SimpleActionServer<ros_motion_builder_msgs::BuildMotionAction> BMServer;
  typedef actionlib::SimpleActionServer<ros_motion_builder_msgs::RunMotionAction> RMServer;
  typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> PMClient;

public:
  ROSMotionBuilderNode();
  bool initialize();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  BMServer action_server_;
  RMServer run_motion_server_;
  PMClient play_motion_client_;
  ros::ServiceServer edit_motion_server_;
  ros::ServiceServer store_motion_server_;
  ros::ServiceServer change_joints_server_;
  ros::ServiceServer list_joints_server_;
  bool running_;

  std::string robot_description_;
  std::string semantic_description_;
  std::vector<std::string> extra_joints_;

  std::unique_ptr<Motion> motion_;

  // ROS Interface methods
  void buildMotionGoalCb();
  void buildMotionPreemptCb();
  void runMotionPreemptCb();
  bool editMotionCb(ros_motion_builder_msgs::EditMotion::Request &req,
                    ros_motion_builder_msgs::EditMotion::Response &res);
  bool storeMotionCb(ros_motion_builder_msgs::StoreMotion::Request &req,
                     ros_motion_builder_msgs::StoreMotion::Response &res);
  bool changeJointsCb(ros_motion_builder_msgs::ChangeJoints::Request &req,
                      ros_motion_builder_msgs::ChangeJoints::Response &res);
  bool listJointGroupsCb(ros_motion_builder_msgs::ListJointGroups::Request &req,
                         ros_motion_builder_msgs::ListJointGroups::Response &res);
  void executeRunMotionCb(const ros_motion_builder_msgs::RunMotionGoalConstPtr &goal);

  // Utility methods
  void motionToROSMsg(ros_motion_builder_msgs::Motion &motion);
};

}  // namespace pal

#endif /* H_ROS_MOTION_BUILDER */
