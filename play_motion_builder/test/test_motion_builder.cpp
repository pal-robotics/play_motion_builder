#include <ros/ros.h>
#include <gtest/gtest.h>

#include <play_motion_builder_msgs/BuildMotionAction.h>
#include <play_motion_builder_msgs/RunMotionAction.h>
#include <play_motion_builder_msgs/EditMotion.h>
#include <play_motion_builder_msgs/StoreMotion.h>
#include <play_motion_builder_msgs/ChangeJoints.h>

#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

#include <unordered_map>
#include <mutex>
#include <fstream>

namespace pal
{
class TestBot
{
public:
  TestBot() : as_(1, &queue_)
  {
    nh_.setCallbackQueue(&queue_);

    joints_["arm_left_joint"] = 0.0;
    joints_["arm_right_joint"] = 0.0;
    joints_["head_joint"] = 0.0;

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    timer_ = nh_.createTimer(ros::Duration(0.05), &TestBot::timerCb, this);

    as_.start();
  }

  void changeJointPos(double left_arm, double right_arm, double head)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);

    joints_["arm_left_joint"] = left_arm;
    joints_["arm_right_joint"] = right_arm;
    joints_["head_joint"] = head;
  }

private:
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner as_;

  ros::Publisher joint_pub_;
  ros::Timer timer_;

  std::unordered_map<std::string, double> joints_;
  std::mutex joint_mutex_;

  void timerCb(const ros::TimerEvent&)
  {
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      for (const auto& joint : joints_)
      {
        js.name.push_back(joint.first);
        js.position.push_back(joint.second);
      }
    }
    joint_pub_.publish(js);
  }
};

template <typename InputIterator1, typename InputIterator2>
bool range_equal(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2,
                 InputIterator2 last2)
{
  while (first1 != last1 && first2 != last2)
  {
    if (*first1 != *first2)
      return false;
    ++first1;
    ++first2;
  }
  return (first1 == last1) && (first2 == last2);
}

bool compare_files(const std::string& filename1, const std::string& filename2)
{
  std::ifstream file1(filename1);
  std::ifstream file2(filename2);

  std::istreambuf_iterator<char> begin1(file1);
  std::istreambuf_iterator<char> begin2(file2);

  std::istreambuf_iterator<char> end;

  return range_equal(begin1, end, begin2, end);
}

// Test not in building mode
TEST(MotionBuilderTest, actionNotStartedTest)
{
  // Call edit service
  play_motion_builder_msgs::EditMotion srv_edit;
  ASSERT_TRUE(ros::service::waitForService("play_motion_builder_node/edit_motion",
                                           ros::Duration(5.0)));
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  EXPECT_FALSE(srv_edit.response.ok);
  EXPECT_EQ("No motion being built", srv_edit.response.message);

  // Call Store service
  play_motion_builder_msgs::StoreMotion srv_store;
  ASSERT_TRUE(ros::service::waitForService("play_motion_builder_node/store_motion",
                                           ros::Duration(5.0)));
  ros::service::call<play_motion_builder_msgs::StoreMotion>(
      "play_motion_builder_node/store_motion", srv_store);
  EXPECT_FALSE(srv_store.response.ok);
  EXPECT_EQ("No motion being built, could not store.", srv_store.response.message);

  // Call change joints service
  play_motion_builder_msgs::ChangeJoints srv_change_joints;
  ASSERT_TRUE(ros::service::waitForService("play_motion_builder_node/change_joints",
                                           ros::Duration(5.0)));
  ros::service::call<play_motion_builder_msgs::ChangeJoints>(
      "play_motion_builder_node/change_joints", srv_change_joints);
  EXPECT_FALSE(srv_change_joints.response.ok);
  EXPECT_EQ("No motion being built", srv_change_joints.response.message);

  // Call play action
  ros::NodeHandle nh;
  play_motion_builder_msgs::RunMotionGoal rmg;
  actionlib::SimpleActionClient<play_motion_builder_msgs::RunMotionAction> client(
      nh, "play_motion_builder_node/run");
  client.waitForServer(ros::Duration(5));
  client.sendGoalAndWait(rmg, ros::Duration(5));
  // Check state
  EXPECT_EQ(actionlib::SimpleClientGoalState::ABORTED, client.getState().state_);
}

TEST(MotionBuilderTest, buildNewMotion)
{
  TestBot tb;

  ros::NodeHandle nh;
  actionlib::SimpleActionClient<play_motion_builder_msgs::BuildMotionAction> client(
      nh, "play_motion_builder_node/build");

  ASSERT_TRUE(client.waitForServer(ros::Duration(5.0))) << "Server didn't start";

  // Send an empty goal.motion to start a new one
  play_motion_builder_msgs::BuildMotionGoal goal;
  client.sendGoal(goal);
  ros::spinOnce();
  ros::Duration(0.1).sleep();  // Wait a bit for the goal to be received

  // Call edit service in LIST model
  play_motion_builder_msgs::EditMotion srv_edit;
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list is empty
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(0, srv_edit.response.motion.keyframes.size());

  // Capture position and check added
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::APPEND;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(1, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[0].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint" ||
        srv_edit.response.motion.joints[i] == "arm_right_joint" ||
        srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.0, srv_edit.response.motion.keyframes[0].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(0.0, srv_edit.response.motion.keyframes[0].time_from_last, 0.001);

  // Move robot and capture
  tb.changeJointPos(0.1, 0.2, 0.3);
  ros::Duration(0.15).sleep();  // Wait a bit for the change to take effect
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::APPEND;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(2, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[0].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint")
    {
      EXPECT_NEAR(0.1, srv_edit.response.motion.keyframes[1].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "arm_right_joint")
    {
      EXPECT_NEAR(0.2, srv_edit.response.motion.keyframes[1].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.3, srv_edit.response.motion.keyframes[1].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(5.0, srv_edit.response.motion.keyframes[1].time_from_last, 0.001);

  // Move robot and edit first
  tb.changeJointPos(0.4, 0.5, 0.6);
  ros::Duration(0.15).sleep();  // Wait a bit for the change to take effect
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::EDIT;
  srv_edit.request.step_id = 0;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(2, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[0].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint")
    {
      EXPECT_NEAR(0.4, srv_edit.response.motion.keyframes[0].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "arm_right_joint")
    {
      EXPECT_NEAR(0.5, srv_edit.response.motion.keyframes[0].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.6, srv_edit.response.motion.keyframes[0].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(5.0, srv_edit.response.motion.keyframes[1].time_from_last, 0.001);

  // Copy motion 0 as last motion
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::COPY_AS_LAST;
  srv_edit.request.step_id = 0;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(3, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[2].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint")
    {
      EXPECT_NEAR(0.4, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "arm_right_joint")
    {
      EXPECT_NEAR(0.5, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.6, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(5.0, srv_edit.response.motion.keyframes[2].time_from_last, 0.001);

  // Copy motion 1 as motion 2
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::COPY_AS_NEXT;
  srv_edit.request.step_id = 1;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(4, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[2].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint")
    {
      EXPECT_NEAR(0.1, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "arm_right_joint")
    {
      EXPECT_NEAR(0.2, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.3, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(5.0, srv_edit.response.motion.keyframes[2].time_from_last, 0.001);

  // Remove motion 2
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::REMOVE;
  srv_edit.request.step_id = 2;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check list has one element
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(3, srv_edit.response.motion.keyframes.size());
  // Review element
  EXPECT_EQ(3, srv_edit.response.motion.joints.size());
  EXPECT_EQ(3, srv_edit.response.motion.keyframes[2].pose.size());
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "arm_left_joint")
    {
      EXPECT_NEAR(0.4, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "arm_right_joint")
    {
      EXPECT_NEAR(0.5, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else if (srv_edit.response.motion.joints[i] == "head_joint")
    {
      EXPECT_NEAR(0.6, srv_edit.response.motion.keyframes[2].pose[i], 0.001);
    }
    else
    {
      EXPECT_TRUE(false) << "Unknown joint added " << srv_edit.response.motion.joints[i];
    }
  }
  EXPECT_NEAR(5.0, srv_edit.response.motion.keyframes[2].time_from_last, 0.001);

  // Test storing a motion
  play_motion_builder_msgs::StoreMotion srv_store;
  srv_store.request.file_path = "/tmp/test_motion_new.yaml";
  srv_store.request.ros_name = "test_motion_new";
  srv_store.request.meta.name = "Test Motion New";
  srv_store.request.meta.usage = "Test";
  srv_store.request.meta.description = "Motion created to test the system";
  ros::service::call<play_motion_builder_msgs::StoreMotion>(
      "play_motion_builder_node/store_motion", srv_store);
  EXPECT_TRUE(srv_store.response.ok) << srv_store.response.message;

  // Compare files
  std::string filepath;
  nh.getParam("test_file_path_new", filepath);
  EXPECT_TRUE(compare_files(srv_store.request.file_path, filepath));

  // Stop motion builder
  client.cancelGoal();
  ros::Duration(0.1).sleep();  // Wait a moment

  // Make sure system is done
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  EXPECT_FALSE(srv_edit.response.ok);
  EXPECT_EQ("No motion being built", srv_edit.response.message);
}

TEST(MotionBuilderTest, editExistingMotion)
{
  TestBot tb;

  ros::NodeHandle nh;
  actionlib::SimpleActionClient<play_motion_builder_msgs::BuildMotionAction> client(
      nh, "play_motion_builder_node/build");

  ASSERT_TRUE(client.waitForServer(ros::Duration(5.0))) << "Server didn't start";

  // Send an empty goal.motion to start a new one
  play_motion_builder_msgs::BuildMotionGoal goal;
  goal.motion = "test_motion_edit";
  client.sendGoal(goal);
  ros::spinOnce();
  ros::Duration(0.1).sleep();  // Wait a bit for the goal to be received

  // Check that the motion was properly loaded
  play_motion_builder_msgs::EditMotion srv_edit;
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  // Check motion is properly loaded
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_EQ(3, srv_edit.response.motion.keyframes.size());
  EXPECT_EQ(2, srv_edit.response.motion.joints.size());

  size_t head_index, arm_left_index;
  for (size_t i = 0; i < srv_edit.response.motion.joints.size(); ++i)
  {
    if (srv_edit.response.motion.joints[i] == "head_joint")
      head_index = i;
    else if (srv_edit.response.motion.joints[i] == "arm_left_joint")
      arm_left_index = i;
  }
  // Check keyframe 1
  EXPECT_NEAR(0.0, srv_edit.response.motion.keyframes[0].time_from_last, 0.001);
  EXPECT_NEAR(0.1, srv_edit.response.motion.keyframes[0].pose[head_index], 0.001);
  EXPECT_NEAR(0.2, srv_edit.response.motion.keyframes[0].pose[arm_left_index], 0.001);
  // Check keyframe 2
  EXPECT_NEAR(1.0, srv_edit.response.motion.keyframes[1].time_from_last, 0.001);
  EXPECT_NEAR(0.11, srv_edit.response.motion.keyframes[1].pose[head_index], 0.001);
  EXPECT_NEAR(0.21, srv_edit.response.motion.keyframes[1].pose[arm_left_index], 0.001);
  // Check keyframe 3
  EXPECT_NEAR(2.0, srv_edit.response.motion.keyframes[2].time_from_last, 0.001);
  EXPECT_NEAR(0.12, srv_edit.response.motion.keyframes[2].pose[head_index], 0.001);
  EXPECT_NEAR(0.22, srv_edit.response.motion.keyframes[2].pose[arm_left_index], 0.001);

  // Add a joint
  play_motion_builder_msgs::ChangeJoints srv_change;
  // Try to change joint inside a group (should fail)
  srv_change.request.joints_to_add.push_back("arm_right_joint");
  ros::service::call<play_motion_builder_msgs::ChangeJoints>(
      "play_motion_builder_node/change_joints", srv_change);
  ASSERT_FALSE(srv_change.response.ok) << srv_change.response.message;
  EXPECT_EQ(2, srv_change.response.used_joints.size());
  EXPECT_EQ("arm_left", srv_change.response.current_group);

  // Change joint group
  srv_change.request.joints_to_add.clear();
  srv_change.request.group = "both_arms";
  ros::service::call<play_motion_builder_msgs::ChangeJoints>(
      "play_motion_builder_node/change_joints", srv_change);
  ASSERT_TRUE(srv_change.response.ok) << srv_change.response.message;
  EXPECT_EQ(3, srv_change.response.used_joints.size());
  EXPECT_EQ("both_arms", srv_change.response.current_group);

  // Remove extra joint
  srv_change.request.joints_to_remove.push_back("head_joint");
  srv_change.request.group = "";
  ros::service::call<play_motion_builder_msgs::ChangeJoints>(
      "play_motion_builder_node/change_joints", srv_change);
  ASSERT_TRUE(srv_change.response.ok) << srv_change.response.message;
  EXPECT_EQ(2, srv_change.response.used_joints.size());
  EXPECT_EQ("both_arms", srv_change.response.current_group);

  // Change time increment of step 2
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::EDIT_TIME;
  srv_edit.request.step_id = 1;
  srv_edit.request.time = 5.5;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  EXPECT_TRUE(srv_edit.response.ok) << srv_edit.response.message;
  EXPECT_NEAR(5.5, srv_edit.response.motion.keyframes[1].time_from_last, 0.001);

  // Test storing a motion
  play_motion_builder_msgs::StoreMotion srv_store;
  srv_store.request.file_path = "/tmp/test_motion_edit.yaml";
  srv_store.request.ros_name = "test_motion_edit";
  ros::service::call<play_motion_builder_msgs::StoreMotion>(
      "play_motion_builder_node/store_motion", srv_store);
  EXPECT_TRUE(srv_store.response.ok) << srv_store.response.message;
  // Check result is fine
  // Compare files
  std::string filepath;
  nh.getParam("test_file_path_edit", filepath);
  EXPECT_TRUE(compare_files(srv_store.request.file_path, filepath));

  // Stop motion builder
  client.cancelGoal();
  ros::Duration(0.1).sleep();  // Wait a moment

  // Make sure system is done
  srv_edit.request.action = play_motion_builder_msgs::EditMotion::Request::LIST;
  ros::service::call<play_motion_builder_msgs::EditMotion>(
      "play_motion_builder_node/edit_motion", srv_edit);
  EXPECT_FALSE(srv_edit.response.ok);
  EXPECT_EQ("No motion being built", srv_edit.response.message);
}

}  // namespace pal

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_motion_builder");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
