cmake_minimum_required(VERSION 2.8.3)
project(carl_interactive_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib interactive_markers rail_pick_and_place_msgs rail_segmentation roscpp sensor_msgs wpi_jaco_msgs)

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
add_executable(carl_interactive_manipulation src/carl_interactive_manipulation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(carl_interactive_manipulation
  ${catkin_LIBRARIES}
)
## Add cmake target dependencies of the executable/library
add_dependencies(${PROJECT_NAME} wpi_jaco_msgs_gencpp rail_pick_and_place_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
