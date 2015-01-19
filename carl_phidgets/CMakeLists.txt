cmake_minimum_required(VERSION 2.8.3)
project(carl_phidgets)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs libphidgets roscpp sensor_msgs std_msgs std_srvs tf)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_api
  CATKIN_DEPENDS geometry_msgs libphidgets roscpp sensor_msgs std_msgs std_srvs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(phidgets_imu src/imu_ros_i.cpp)
add_library(phidgets_api src/phidget.cpp src/imu.cpp)

## Declare a cpp executable
add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)
add_executable(orientation_filter src/orientation_filter.cpp)

add_dependencies(phidgets_imu phidgets_api)

## Specify libraries to link a library or executable target against
target_link_libraries(phidgets_imu ${catkin_LIBRARIES} phidgets_api)
target_link_libraries(phidgets_imu_node ${catkin_LIBRARIES} phidgets_imu)
target_link_libraries(phidgets_api ${catkin_LIBRARIES})
target_link_libraries(orientation_filter ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS phidgets_api phidgets_imu phidgets_imu_node orientation_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
