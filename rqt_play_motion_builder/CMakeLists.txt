cmake_minimum_required(VERSION 3.5)
project(rqt_play_motion_builder)

find_package(ament_cmake_auto REQUIRED)
find_package(Qt5Widgets REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(rqt_play_motion_builder_MOCS
  include/${PROJECT_NAME}/rqt_play_motion_builder.hpp
  include/${PROJECT_NAME}/properties_dialog.hpp
)
qt5_wrap_ui(rqt_play_motion_builder_UIS_H
  resources/rqt_play_motion_builder.ui
  resources/properties_dialog.ui
)

ament_auto_add_library(${PROJECT_NAME}_gui SHARED
  src/rqt_play_motion_builder.cpp
  src/properties_dialog.cpp
  ${rqt_play_motion_builder_MOCS}
  ${rqt_play_motion_builder_UIS_H}
)
target_link_libraries(${PROJECT_NAME}_gui Qt5::Widgets)

# class_loader_hide_library_symbols(${PROJECT_NAME}_gui)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()

