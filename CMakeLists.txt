cmake_minimum_required(VERSION 3.10)
project(mrs_gazebo_extras_resources)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O3 -g -Wall")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  std_msgs
  mrs_msgs
  tf2_msgs
  tf2
  tf2_ros
  geometry_msgs
  cv_bridge
  mrs_lib
  )


# include Gazebo
find_package(gazebo REQUIRED)

# include OpenCV
find_package(OpenCV REQUIRED HINTS /usr/local/lib)

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS GAZEBO Boost OpenCV
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros std_msgs mrs_msgs tf2_msgs tf2 tf2_ros geometry_msgs mrs_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(MRSGazeboLinkAttacherPlugin src/link_attacher.cpp)
target_link_libraries(MRSGazeboLinkAttacherPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  )

add_library(MRSGazeboThermalCameraPlugin src/thermal_camera.cpp)
target_link_libraries(MRSGazeboThermalCameraPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  CameraPlugin
  )
