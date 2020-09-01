cmake_minimum_required(VERSION 2.8.3)
project(ros_motion_builder)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    sensor_msgs
    ros_motion_builder_msgs
    actionlib
    play_motion_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
LIBRARIES motion_model_lib 
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(motion_model_lib src/motion_model.cpp)

add_executable(ros_motion_builder_node 
	src/ros_motion_builder_node.cpp
	src/ros_motion_builder.cpp)

target_link_libraries(motion_model_lib ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(ros_motion_builder_node ${catkin_LIBRARIES} motion_model_lib)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS motion_model_lib 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_motion_model
    test/test_motion_model.test
    test/test_motion_model.cpp)
  target_link_libraries(test_motion_model motion_model_lib)

  add_rostest_gtest(test_motion_builder
    test/test_motion_builder.test
    test/test_motion_builder.cpp)
  target_link_libraries(test_motion_builder ${catkin_LIBRARIES})
endif()

