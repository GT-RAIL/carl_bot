cmake_minimum_required(VERSION 2.8.3)
project(carl_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp joy geometry_msgs)


###################################################
## Declare things to be passed to other projects ##
###################################################


catkin_package()

###########
## Build ##
###########


include_directories(
	include
 	${catkin_INCLUDE_DIRS}	
 	${Boost_INCLUDE_DIR}
)

set(CMAKE_BUILD_TYPE Release)

## Declare a cpp executable
add_executable(carl_joy_teleop src/carl_joy_teleop.cpp)


target_link_libraries(carl_joy_teleop ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS carl_joy_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)