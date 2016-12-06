## BEGIN_TUTORIAL
## This CMakeLists.txt file for tno_rviz_plugins builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugins)
find_package(catkin REQUIRED COMPONENTS rviz automotive_sensor_msgs)
catkin_package()

# Setup includes
include_directories(
	src/visuals 
	src/displays 
	${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# C++11
set(CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS}")

set(SOURCE_FILES

        src/visuals/line_visual.cpp
        src/visuals/line_visual.h
        src/displays/line_info_display.cpp
        src/displays/line_info_display.h
  #src/displays/moc_line_info_display.cxx # This file can be obtained by running moc-qt4 line_info_display.h > moc_line_info_display.cxx

)

# Create QT Plugins
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
