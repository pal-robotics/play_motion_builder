cmake_minimum_required(VERSION 3.0.2)
project(rqt_play_motion_builder)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rqt_gui_cpp
  actionlib
  play_motion_msgs
  urdf
  sensor_msgs
  play_motion_builder
  play_motion_builder_msgs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_gui
#  CATKIN_DEPENDS actionlib play_motion_builder_msgs
#  DEPENDS system_lib
)
catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_play_motion_builder_MOCS
    include/${PROJECT_NAME}/rqt_play_motion_builder.h
    include/${PROJECT_NAME}/properties_dialog.h)
  qt5_wrap_ui(rqt_play_motion_builder_UIS_H
    resources/rqt_play_motion_builder.ui
    resources/properties_dialog.ui)
else()
  qt4_wrap_cpp(rqt_play_motion_builder_MOCS
    include/${PROJECT_NAME}/rqt_play_motion_builder.h
    include/${PROJECT_NAME}/properties_dialog.h)
  qt4_wrap_ui(rqt_play_motion_builder_UIS_H
    resources/rqt_play_motion_builder.ui
    resources/properties_dialog.ui)
endif()

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_gui
  src/rqt_play_motion_builder.cpp
  src/properties_dialog.cpp
  ${rqt_play_motion_builder_MOCS}
  ${rqt_play_motion_builder_UIS_H}
)

target_link_libraries(${PROJECT_NAME}_gui ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME}_gui Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME}_gui ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader REQUIRED)
class_loader_hide_library_symbols(${PROJECT_NAME}_gui)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/rqt_play_motion_builder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_play_motion_builder.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
