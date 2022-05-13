cmake_minimum_required(VERSION 2.8.3)
project(marker_mapping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  geometry_msgs
  robotnik_msgs
  rospy
  std_msgs
  message_generation
  genmsg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
   FILES
      MarkerMappingState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SaveMarker.srv
   InitPoseFromMarker.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs robotnik_msgs
)

catkin_package(
)

