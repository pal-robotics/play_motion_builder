cmake_minimum_required(VERSION 3.5)
project(play_motion_builder)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_compile_definitions(RCLCPP_VERSION_MAJOR=${rclcpp_VERSION_MAJOR})

# motion_model_lib
ament_auto_add_library(motion_model_lib SHARED
  src/motion_model.cpp
)
target_link_libraries(motion_model_lib yaml-cpp)

# play_motion_builder_node
ament_auto_add_executable(play_motion_builder_node
  src/play_motion_builder_node.cpp
  src/play_motion_builder.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  # ament_auto_add_gtest(test_motion_model
  #   test/test_motion_model.test
  #   test/test_motion_model.cpp
  # )

  # ament_auto_add_gtest(test_motion_builder
  #   test/test_motion_builder.test
  #   test/test_motion_builder.cpp
  # )
endif()

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()

