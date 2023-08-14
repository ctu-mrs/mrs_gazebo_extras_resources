cmake_minimum_required(VERSION 3.5)
project(mrs_gazebo_extras_resources)

set(CATKIN_DEPENDENCIES
  cv_bridge
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  mrs_lib
  mrs_msgs
  roscpp
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# include Gazebo
find_package(gazebo REQUIRED)
ign_import_target(UUID)

# include OpenCV
find_package(OpenCV REQUIRED)

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

set(LIBRARIES
  MrsGazeboExtrasResources_AttacherPlugin MrsGazeboExtrasResources_ThermalCameraPlugin
  )

catkin_package(
  DEPENDS GAZEBO Boost
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(MrsGazeboExtrasResources_AttacherPlugin
  src/link_attacher.cpp
  )

target_link_libraries(MrsGazeboExtrasResources_AttacherPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  )

add_library(MrsGazeboExtrasResources_ThermalCameraPlugin
  src/thermal_camera.cpp
  )

target_link_libraries(MrsGazeboExtrasResources_ThermalCameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  CameraPlugin
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
