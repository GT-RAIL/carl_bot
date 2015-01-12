cmake_minimum_required(VERSION 2.8.3)

project(carl_phidgets)

find_package(catkin REQUIRED COMPONENTS geometry_msgs libphidgets nodelet roscpp sensor_msgs std_msgs std_srvs tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_api
  CATKIN_DEPENDS geometry_msgs libphidgets nodelet roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(phidget21 SHARED IMPORTED)
#set_target_properties(phidget21 PROPERTIES IMPORTED_LOCATION "${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so")
add_library(phidgets_imu src/imu_ros_i.cpp)
add_library(phidgets_imu_nodelet src/phidgets_imu_nodelet.cpp)
add_library(phidgets_api src/phidget.cpp
                         src/imu.cpp)

add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)

add_dependencies(phidgets_imu phidgets_api)

target_link_libraries(phidgets_imu ${catkin_LIBRARIES} phidgets_api)
target_link_libraries(phidgets_imu_nodelet ${catkin_LIBRARIES} phidgets_imu)
target_link_libraries(phidgets_imu_node ${catkin_LIBRARIES} phidgets_imu)

target_link_libraries(phidgets_api ${catkin_LIBRARIES})

install(TARGETS phidgets_api phidgets_imu phidgets_imu_nodelet phidgets_imu_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES phidgets_imu_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
   ${CMAKE_CURRENT_SOURCE_DIR}/share/setup-udev.sh
   ${install_dir}/src/libphidget21/udev/99-phidgets.rules
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
