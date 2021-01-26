cmake_minimum_required(VERSION 2.8.3)
project(lane_marker_messages)

###########
## SETUP ##
###########

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Point.msg
  LaneMarker.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  ## Any other messages that this package depends on
)


############
## CATKIN ##
############

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

###########
## Build ##
###########

## No specific library build associated with this package
