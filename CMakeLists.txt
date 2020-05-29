cmake_minimum_required(VERSION 2.8.3)
project(sample_grasp_executor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rail_manipulation_msgs
  roscpp
  tf
  tf_conversions
  tf2_geometry_msgs
  tf2_ros
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(sample_grasp_executor src/SampleGraspExecutor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sample_grasp_executor ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(sample_grasp_executor rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sample_grasp_executor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
