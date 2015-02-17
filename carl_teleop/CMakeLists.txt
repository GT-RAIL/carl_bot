cmake_minimum_required(VERSION 2.8.3)
project(carl_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib joy roscpp sensor_msgs std_msgs std_srvs wpi_jaco_msgs)

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
add_executable(carl_joy_teleop src/carl_joy_teleop.cpp)
add_dependencies(carl_joy_teleop wpi_jaco_msgs_gencpp)
target_link_libraries(carl_joy_teleop ${catkin_LIBRARIES})

add_executable(carl_key_teleop src/carl_key_teleop.cpp)
add_dependencies(carl_key_teleop wpi_jaco_msgs_gencpp)
target_link_libraries(carl_key_teleop ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS carl_joy_teleop carl_key_teleop
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
