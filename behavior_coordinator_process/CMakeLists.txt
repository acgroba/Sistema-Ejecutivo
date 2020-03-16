cmake_minimum_required(VERSION 2.8.3)
project(behavior_coordinator_process)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
add_definitions(-std=c++11)
add_definitions(-g)

# Directories definition
set(BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR
  src/source
)

set(BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR
  src/include
)

set(BEHAVIOR_COORDINATOR_PROCESS_TEST_DIR
  src/test
)

# Files declaration
set(BEHAVIOR_COORDINATOR_PROCESS_SOURCE_FILES
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_coordinator_process.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_activation_changes.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_coordinator_process_main.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/argument_descriptor.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_catalog.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_descriptor.cpp
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/prettify.cpp
)

set(BEHAVIOR_COORDINATOR_PROCESS_HEADER_FILES
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/behavior_coordinator_process.h
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/behavior_activation_changes.h
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/argument_descriptor.h
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/behavior_catalog.h
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/behavior_descriptor.h
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}/prettify.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  robot_process
  aerostack_msgs
  droneMsgsROS #legacy
  
  

)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  robot_process
  aerostack_msgs
  droneMsgsROS # legacy
  DEPENDS yaml-cpp

)

###########
## Build ##
###########
include_directories(
  ${BEHAVIOR_COORDINATOR_PROCESS_INCLUDE_DIR}
  ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_coordinator_process_lib ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_FILES} ${BEHAVIOR_COORDINATOR_PROCESS_HEADER_FILES})
add_dependencies(behavior_coordinator_process_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_coordinator_process_lib ${catkin_LIBRARIES})

add_executable(behavior_coordinator_process ${BEHAVIOR_COORDINATOR_PROCESS_SOURCE_DIR}/behavior_coordinator_process_main.cpp)
add_dependencies(behavior_coordinator_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_coordinator_process behavior_coordinator_process_lib)
target_link_libraries(behavior_coordinator_process ${catkin_LIBRARIES})
target_link_libraries(behavior_coordinator_process yaml-cpp)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(behavior_coordinator_process_test ${BEHAVIOR_COORDINATOR_PROCESS_TEST_DIR}/behavior_coordinator_process_test.cpp)
  target_link_libraries(behavior_coordinator_process_test behavior_coordinator_process_lib)
  target_link_libraries(behavior_coordinator_process_test ${catkin_LIBRARIES})

endif()


