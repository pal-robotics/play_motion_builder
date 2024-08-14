cmake_minimum_required(VERSION 3.8)
project(play_motion_builder_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BuildMotion.action"
  "action/RunMotion.action"
  "msg/Frame.msg"
  "msg/Meta.msg"
  "msg/Motion.msg"
  "srv/ChangeJoints.srv"
  "srv/EditMotion.srv"
  "srv/ListJointGroups.srv"
  "srv/StoreMotion.srv"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
