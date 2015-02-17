cmake_minimum_required(VERSION 2.8.3)
project(carl_dynamixel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp dynamixel_msgs sensor_msgs tf)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LookAtFrame.srv
  LookAtPoint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
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
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(back_joints_node src/back_joint_node.cpp)
add_executable(servo_pan_tilt src/servo_pan_tilt.cpp)
add_executable(front_joints_node src/front_joint_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(back_joints_node
  ${catkin_LIBRARIES}
)
target_link_libraries(servo_pan_tilt
  ${catkin_LIBRARIES}
)

target_link_libraries(front_joints_node
  ${catkin_LIBRARIES}
)

add_dependencies(servo_pan_tilt ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS back_joints_node front_joints_node servo_pan_tilt
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
