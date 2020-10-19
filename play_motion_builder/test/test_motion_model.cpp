#include <play_motion_builder/motion_model.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>

namespace pal
{
TEST(MotionBuilderModelTest, jointLimitTest)
{
  JointModel jm;              // Empty constructor defined
  JointModel jm2(-1.0, 1.0);  // Constructor with values

  double valid_pos = 0.0;
  double too_low_pos = -2.0;
  double too_big_pos = 2.0;

  EXPECT_TRUE(jm2.inLimits(valid_pos));
  EXPECT_FALSE(jm2.inLimits(too_low_pos));
  EXPECT_FALSE(jm2.inLimits(too_big_pos));
}

TEST(MotionBuilderModelTest, keyframeTest)
{
  // Empty keyframe
  KeyFrame k(Motion::DEFAULT_TIME);

  k.addPosition("Test1", 1.0);

  EXPECT_EQ(Motion::DEFAULT_TIME, k.getTime());
  EXPECT_EQ(1.0, k.getJointPosition("Test1"));
  EXPECT_TRUE(std::isnan(k.getJointPosition("Test2")));

  // copy constructor
  KeyFrame k2(k);

  EXPECT_EQ(k.getTime(), k2.getTime());
  EXPECT_EQ(1.0, k2.getJointPosition("Test1"));
  EXPECT_TRUE(std::isnan(k2.getJointPosition("Test2")));

  k2.addPosition("Test2", 1.0);
  std::map<std::string, bool> used;
  used["Test1"] = false;
  used["Test2"] = true;
  k2.cleanUnused(used);

  EXPECT_TRUE(std::isnan(k2.getJointPosition("Test1")));
  EXPECT_EQ(1.0, k2.getJointPosition("Test2"));

  k2.addPosition("Test1", 1.0);

  std::vector<std::string> joint_list;
  joint_list.push_back("Test1");

  // Test print point element
  YAML::Emitter em;
  em << k2.print(0.0, 1.0, joint_list);  // Basic copy;
  EXPECT_EQ("time_from_start: 5\npositions: [1]", std::string(em.c_str()));

  YAML::Emitter em2;
  em2 << k2.print(1.0, 1.0, joint_list);  // No downshift, basetime 1
  EXPECT_EQ("time_from_start: 6\npositions: [1]", std::string(em2.c_str()));

  YAML::Emitter em3;
  em3 << k2.print(0.0, 2.0, joint_list);  // Downshift 2, basetime 0
  EXPECT_EQ("time_from_start: 10\npositions: [1]", std::string(em3.c_str()));

  YAML::Emitter em4;
  em4 << k2.print(1.0, 2.0, joint_list);  // Downshift 2, basetime 1
  EXPECT_EQ("time_from_start: 11\npositions: [1]", std::string(em4.c_str()));

  // Test print motion to frame
  YAML::Emitter em5;
  em5 << k2.print(joint_list);
  EXPECT_EQ("joints: [Test1]\npoints:\n  - time_from_start: 0\n    "
            "positions: [1]",
            std::string(em5.c_str()));
}

TEST(MotionBuilderModelTest, motionTest)
{
  Motion m("", "", {});

  // Test loading joints
  EXPECT_FALSE(m.jointsLoaded());
  for (int i = 0; i < 3; ++i)
  {
    JointModel jm(0.0, 3.14159226);
    m.addJointModel("Test_joint_" + std::to_string(i + 1), jm);
  }
  EXPECT_FALSE(m.jointsLoaded());
  // Test loading groups
  for (int i = 0; i < 3; ++i)
  {
    m.addJointToGroup("TestGroup", "Test_joint_" + std::to_string(i + 1));
  }
  EXPECT_TRUE(m.jointsLoaded());
  m.setCurrentGroup("TestGroup");
  EXPECT_EQ(3, m.getJoints().size());

  // Test loading frames
  sensor_msgs::JointState js1;
  js1.name.push_back("Test_joint_1");
  js1.position.push_back(1.1);
  js1.name.push_back("Test_joint_2");
  js1.position.push_back(1.2);
  js1.name.push_back("Test_joint_3");
  js1.position.push_back(1.3);
  js1.name.push_back("Test_joint_4");
  js1.position.push_back(1.4);
  sensor_msgs::JointStateConstPtr js1c(new sensor_msgs::JointState(js1));
  m.addKeyFrame(js1c);  // Add with default time
  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime()) << "Should be 0 as it's first "
                                                "frame";
  EXPECT_EQ(1.1, m.getKeyFrame(0).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(0).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(0).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(0).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not on used list";

  m.addKeyFrame(js1c);  // Add with default time
  EXPECT_EQ(Motion::DEFAULT_TIME, m.getKeyFrame(1).getTime());
  EXPECT_EQ(1.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(1).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(1).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(1).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not on used list";

  // Test updating a frame
  sensor_msgs::JointState js2;
  js2.name.push_back("Test_joint_1");
  js2.position.push_back(2.1);
  js2.name.push_back("Test_joint_2");
  js2.position.push_back(2.2);
  js2.name.push_back("Test_joint_3");
  js2.position.push_back(2.3);
  js2.name.push_back("Test_joint_4");
  js2.position.push_back(2.4);
  sensor_msgs::JointStateConstPtr js2c(new sensor_msgs::JointState(js2));

  m.updateKeyFrame(js2c, 1);
  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime()) << "Should be 0 as it's first "
                                                "frame";
  EXPECT_EQ(1.1, m.getKeyFrame(0).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(0).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(0).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(0).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(Motion::DEFAULT_TIME, m.getKeyFrame(1).getTime());
  EXPECT_EQ(2.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));
  EXPECT_EQ(2.2, m.getKeyFrame(1).getJointPosition("Test_joint_2"));
  EXPECT_EQ(2.3, m.getKeyFrame(1).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(1).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";

  // Test changing the time
  EXPECT_THROW(m.changeTime(5, 1.0), ros::Exception) << "Trying to modify a non-existing "
                                                        "frame should result in an Exception";
  m.changeTime(1, 2.0);
  EXPECT_EQ(2.0, m.getKeyFrame(1).getTime());

  // Test changing joint positions
  EXPECT_THROW(m.changeJoint(5, "", 1.0), ros::Exception)
      << "Trying to modify a non-existing frame should result in an Exception";
  EXPECT_THROW(m.changeJoint(1, "Non-existing", 1.0), ros::Exception)
      << "Trying to modify a non-existing joint should result in an Exception";
  double res = m.changeJoint(1, "Test_joint_1", 5000.0);
  EXPECT_EQ(2.1, res);
  EXPECT_EQ(2.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"))
      << "If value not in joint limits no change";
  res = m.changeJoint(1, "Test_joint_1", 3.1);
  EXPECT_EQ(3.1, res);
  EXPECT_EQ(3.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));

  // Test copying frames
  m.copyFrame(0);  // Copy to last
  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime()) << "Should be 0 as it's first frame";
  EXPECT_EQ(1.1, m.getKeyFrame(0).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(0).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(0).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(0).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(2.0, m.getKeyFrame(1).getTime());
  EXPECT_EQ(3.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));
  EXPECT_EQ(2.2, m.getKeyFrame(1).getJointPosition("Test_joint_2"));
  EXPECT_EQ(2.3, m.getKeyFrame(1).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(1).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(Motion::DEFAULT_TIME, m.getKeyFrame(2).getTime());
  EXPECT_EQ(1.1, m.getKeyFrame(2).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(2).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(2).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(2).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";

  m.copyFrame(1, 2);  // Copy frame 1 as frame 2
  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime()) << "Should be 0 as it's first frame";
  EXPECT_EQ(1.1, m.getKeyFrame(0).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(0).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(0).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(0).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(2.0, m.getKeyFrame(1).getTime());
  EXPECT_EQ(3.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));
  EXPECT_EQ(2.2, m.getKeyFrame(1).getJointPosition("Test_joint_2"));
  EXPECT_EQ(2.3, m.getKeyFrame(1).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(1).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(2.0, m.getKeyFrame(2).getTime());
  EXPECT_EQ(3.1, m.getKeyFrame(2).getJointPosition("Test_joint_1"));
  EXPECT_EQ(2.2, m.getKeyFrame(2).getJointPosition("Test_joint_2"));
  EXPECT_EQ(2.3, m.getKeyFrame(2).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(2).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(Motion::DEFAULT_TIME, m.getKeyFrame(3).getTime());
  EXPECT_EQ(1.1, m.getKeyFrame(3).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(3).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(3).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(3).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";

  // Test removing frames
  EXPECT_EQ(4, m.size());
  m.removeKeyFrame(1);
  m.removeKeyFrame(1);  // Remove 2 keyframes
  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime()) << "Should be 0 as it's first frame";
  EXPECT_EQ(1.1, m.getKeyFrame(0).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(0).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(0).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(0).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(Motion::DEFAULT_TIME, m.getKeyFrame(1).getTime());
  EXPECT_EQ(1.1, m.getKeyFrame(1).getJointPosition("Test_joint_1"));
  EXPECT_EQ(1.2, m.getKeyFrame(1).getJointPosition("Test_joint_2"));
  EXPECT_EQ(1.3, m.getKeyFrame(1).getJointPosition("Test_joint_3"));
  EXPECT_TRUE(std::isnan(m.getKeyFrame(1).getJointPosition("Test_joint_4")))
      << "Should be NaN as it is not used";
  EXPECT_EQ(2, m.size());

  // Test transformation to yaml
  m.changeTime(1, 2.0);
  m.addKeyFrame(js1c);

  YAML::Emitter em;
  em << m.print();  // Basic output
  EXPECT_EQ("joints: [Test_joint_1, Test_joint_2, Test_joint_3]\n"
            "points:\n  "
            "- time_from_start: 0\n    "
            "positions: [1.1, 1.2, 1.3]\n  "
            "- time_from_start: 2\n    "
            "positions: [1.1, 1.2, 1.3]\n  "
            "- time_from_start: 7\n    "
            "positions: [1.1, 1.2, 1.3]",
            std::string(em.c_str()));

  YAML::Emitter em2;
  em2 << m.print(2.0);  // Downshift 2
  EXPECT_EQ("joints: [Test_joint_1, Test_joint_2, Test_joint_3]\n"
            "points:\n  "
            "- time_from_start: 0\n    "
            "positions: [1.1, 1.2, 1.3]\n  "
            "- time_from_start: 4\n    "
            "positions: [1.1, 1.2, 1.3]\n  "
            "- time_from_start: 14\n    "
            "positions: [1.1, 1.2, 1.3]",
            std::string(em2.c_str()));
}

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

TEST(MotionBuilderModelTest, motionFromParamTest)
{
  XmlRpc::XmlRpcValue param_to_load;
  ros::NodeHandle nh;
  nh.getParam("/play_motion/motions/test_motion", param_to_load);

  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, param_to_load.getType());
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, param_to_load["joints"].getType());
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, param_to_load["points"].getType());

  Motion m(param_to_load,
           "<robot name=\"testbot\"><joint name=\"arm_left_1_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_2_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_3_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_4_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_5_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_6_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_left_7_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_1_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_2_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_3_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_4_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_5_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_6_joint\" type=\"revolute\"/>"
           "<joint name=\"arm_right_7_joint\" type=\"revolute\"/>"
           "<joint name=\"torso_1_joint\" type=\"revolute\"/>"
           "<joint name=\"torso_2_joint\" type=\"revolute\"/></robot>",
           "<?xml version=\"1.0\" ?><robot name=\"testbot\"><group name=\"arm_left\">"
           "<joint name=\"arm_left_1_joint\" /><joint name=\"arm_left_2_joint\" />"
           "<joint name=\"arm_left_3_joint\"/><joint name=\"arm_left_4_joint\" />"
           "<joint name=\"arm_left_5_joint\"/><joint name=\"arm_left_6_joint\"/>"
           "<joint name=\"arm_left_7_joint\"/></group><group name=\"arm_right\">"
           "<joint name=\"arm_right_1_joint\" /><joint name=\"arm_right_2_joint\" />"
           "<joint name=\"arm_right_3_joint\" /><joint name=\"arm_right_4_joint\" />"
           "<joint name=\"arm_right_5_joint\"/><joint name=\"arm_right_6_joint\"/>"
           "<joint name=\"arm_right_7_joint\"/></group><group name=\"torso\">"
           "<joint name=\"torso_1_joint\"/><joint name=\"torso_2_joint\"/></group>"
           "<group name=\"both_arms\"><group name=\"arm_left\" /><group name=\"arm_right\" />"
           "</group><group name=\"both_arms_torso\"><group name=\"arm_left\" />"
           "<group name=\"arm_right\" /><group name=\"torso\" /></group></robot>",
           { "head_1_joint", "head_2_joint" });

  // Check correct group selected
  EXPECT_EQ("both_arms_torso", m.getCurrentGroup());

  // Validate groups
  m.setCurrentGroup("arm_left");
  EXPECT_EQ(7, m.getJoints().size());
  EXPECT_EQ("arm_left_1_joint", m.getJoints()[0]);
  EXPECT_EQ("arm_left_2_joint", m.getJoints()[1]);
  EXPECT_EQ("arm_left_3_joint", m.getJoints()[2]);
  EXPECT_EQ("arm_left_4_joint", m.getJoints()[3]);
  EXPECT_EQ("arm_left_5_joint", m.getJoints()[4]);
  EXPECT_EQ("arm_left_6_joint", m.getJoints()[5]);
  EXPECT_EQ("arm_left_7_joint", m.getJoints()[6]);

  m.setCurrentGroup("arm_right");
  EXPECT_EQ(7, m.getJoints().size());
  EXPECT_EQ("arm_right_1_joint", m.getJoints()[0]);
  EXPECT_EQ("arm_right_2_joint", m.getJoints()[1]);
  EXPECT_EQ("arm_right_3_joint", m.getJoints()[2]);
  EXPECT_EQ("arm_right_4_joint", m.getJoints()[3]);
  EXPECT_EQ("arm_right_5_joint", m.getJoints()[4]);
  EXPECT_EQ("arm_right_6_joint", m.getJoints()[5]);
  EXPECT_EQ("arm_right_7_joint", m.getJoints()[6]);

  m.setCurrentGroup("torso");
  EXPECT_EQ(2, m.getJoints().size());
  EXPECT_EQ("torso_1_joint", m.getJoints()[0]);
  EXPECT_EQ("torso_2_joint", m.getJoints()[1]);

  m.setCurrentGroup("both_arms");
  EXPECT_EQ(14, m.getJoints().size());
  EXPECT_EQ("arm_left_1_joint", m.getJoints()[0]);
  EXPECT_EQ("arm_left_2_joint", m.getJoints()[1]);
  EXPECT_EQ("arm_left_3_joint", m.getJoints()[2]);
  EXPECT_EQ("arm_left_4_joint", m.getJoints()[3]);
  EXPECT_EQ("arm_left_5_joint", m.getJoints()[4]);
  EXPECT_EQ("arm_left_6_joint", m.getJoints()[5]);
  EXPECT_EQ("arm_left_7_joint", m.getJoints()[6]);
  EXPECT_EQ("arm_right_1_joint", m.getJoints()[7]);
  EXPECT_EQ("arm_right_2_joint", m.getJoints()[8]);
  EXPECT_EQ("arm_right_3_joint", m.getJoints()[9]);
  EXPECT_EQ("arm_right_4_joint", m.getJoints()[10]);
  EXPECT_EQ("arm_right_5_joint", m.getJoints()[11]);
  EXPECT_EQ("arm_right_6_joint", m.getJoints()[12]);
  EXPECT_EQ("arm_right_7_joint", m.getJoints()[13]);

  m.setCurrentGroup("both_arms_torso");
  EXPECT_EQ(16, m.getJoints().size());
  EXPECT_EQ("arm_left_1_joint", m.getJoints()[0]);
  EXPECT_EQ("arm_left_2_joint", m.getJoints()[1]);
  EXPECT_EQ("arm_left_3_joint", m.getJoints()[2]);
  EXPECT_EQ("arm_left_4_joint", m.getJoints()[3]);
  EXPECT_EQ("arm_left_5_joint", m.getJoints()[4]);
  EXPECT_EQ("arm_left_6_joint", m.getJoints()[5]);
  EXPECT_EQ("arm_left_7_joint", m.getJoints()[6]);
  EXPECT_EQ("arm_right_1_joint", m.getJoints()[7]);
  EXPECT_EQ("arm_right_2_joint", m.getJoints()[8]);
  EXPECT_EQ("arm_right_3_joint", m.getJoints()[9]);
  EXPECT_EQ("arm_right_4_joint", m.getJoints()[10]);
  EXPECT_EQ("arm_right_5_joint", m.getJoints()[11]);
  EXPECT_EQ("arm_right_6_joint", m.getJoints()[12]);
  EXPECT_EQ("arm_right_7_joint", m.getJoints()[13]);
  EXPECT_EQ("torso_1_joint", m.getJoints()[14]);
  EXPECT_EQ("torso_2_joint", m.getJoints()[15]);

  EXPECT_EQ(4, m.size()) << "There should be 4 keyframes";

  EXPECT_EQ(0.0, m.getKeyFrame(0).getTime());
  EXPECT_EQ(3.0, m.getKeyFrame(1).getTime());
  EXPECT_EQ(3.0, m.getKeyFrame(2).getTime());
  EXPECT_EQ(3.0, m.getKeyFrame(3).getTime());

  // Check extends works properly
  sensor_msgs::JointState js;
  js.name.push_back("head_1_joint");
  js.position.push_back(0.0);
  js.name.push_back("head_2_joint");
  js.position.push_back(0.0);
  js.name.push_back("");
  sensor_msgs::JointStateConstPtr jsc(new sensor_msgs::JointState(js));

  JointModel jm(0.0, 3.14159226);
  m.addJointModel("head_1_joint", jm);  // Not used
  m.addJointModel("head_2_joint", jm);  // Not used
  EXPECT_FALSE(m.isJointUsed("head_1_joint"));
  EXPECT_FALSE(m.isJointUsed("head_2_joint"));

  m.extendFrames(jsc);
  EXPECT_FALSE(std::isnan(m.getKeyFrame(0).getJointPosition("head_1_joint")));
  EXPECT_FALSE(std::isnan(m.getKeyFrame(0).getJointPosition("head_2_joint")));
  EXPECT_FALSE(std::isnan(m.getKeyFrame(3).getJointPosition("head_1_joint")));
  EXPECT_FALSE(std::isnan(m.getKeyFrame(3).getJointPosition("head_2_joint")));

  // Output to tmp file to check
  YAML::Emitter em;
  em << YAML::BeginMap << YAML::Key << "play_motion" << YAML::Value << YAML::BeginMap
     << YAML::Key << "motions" << YAML::Value << YAML::BeginMap << YAML::Key
     << "test_motion" << YAML::Value
     << m.print(param_to_load["meta"]["name"], param_to_load["meta"]["usage"],
                param_to_load["meta"]["description"])
     << YAML::EndMap << YAML::EndMap << YAML::EndMap;

  std::ofstream ofile("/tmp/tm.yaml");
  ofile << em.c_str();
  ofile.close();

  // Compare files
  std::string filepath;
  nh.getParam("test_file_path", filepath);
  EXPECT_TRUE(compare_files("/tmp/tm.yaml", filepath));
}
}  // namespace pal

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_motion_model");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
