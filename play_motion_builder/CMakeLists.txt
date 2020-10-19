cmake_minimum_required(VERSION 2.8.3)
project(play_motion_builder)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    sensor_msgs
    play_motion_builder_msgs
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

add_executable(play_motion_builder_node 
	src/play_motion_builder_node.cpp
	src/play_motion_builder.cpp)

target_link_libraries(motion_model_lib ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(play_motion_builder_node ${catkin_LIBRARIES} motion_model_lib)

#############
## Install ##
#############

install(TARGETS play_motion_builder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_model_lib 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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

