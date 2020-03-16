cmake_minimum_required(VERSION 2.8.3)
project(basic_quadrotor_behaviors)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)

# Directories definition
set(BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR
  src/source
)

set(BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR
  src/include
)

set(BASIC_QUADROTOR_BEHAVIORS_HEADER_FILES
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_take_off.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_localize_with_odometry.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_self_localize_with_odometry_1.h        
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_land.h
        ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}/behavior_wait.h
)

set(BASIC_QUADROTOR_BEHAVIORS_SOURCE_FILES
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_take_off.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_localize_with_odometry.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_self_localize_with_odometry_1.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_land.cpp
        ${BASIC_QUADROTOR_BEHAVIORS_SOURCE_DIR}/behavior_wait.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  behavior_execution_controller
  droneMsgsROS
  geometry_msgs
  aerostack_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BASIC_QUADROTOR_BEHAVIORS_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
  std_msgs
  drone_process
  behavior_execution_controller
  droneMsgsROS
  geometry_msgs
  aerostack_msgs
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(basic_quadrotor_behaviors
  src/source/behavior_take_off.cpp
  src/source/behavior_localize_with_odometry.cpp
  src/source/behavior_self_localize_with_odometry_1.cpp
  src/source/behavior_land.cpp
  src/source/behavior_wait.cpp
)
target_link_libraries(basic_quadrotor_behaviors ${catkin_LIBRARIES})
target_link_libraries(basic_quadrotor_behaviors yaml-cpp)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(basic_quadrotor_behaviors ${catkin_EXPORTED_LIBRARIES})
endif()
