cmake_minimum_required(VERSION 2.8.3)
project(play_motion_builder_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Frame.msg
  Motion.msg
  Meta.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EditMotion.srv
  StoreMotion.srv
  ChangeJoints.srv
  ListJointGroups.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  BuildMotion.action
  RunMotion.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)
