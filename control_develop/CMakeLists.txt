cmake_minimum_required(VERSION 3.0.2)
project(control_develop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs sensor_msgs trajectory_msgs rospy
  LIBRARIES
)

add_definitions(-std=c++11)

add_executable(keyboard_control src/keyboard_control.cpp)
add_dependencies(keyboard_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_control ${catkin_LIBRARIES})

add_executable(pub_velosity src/pub_velosity.cpp)
add_dependencies(pub_velosity ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_velosity ${catkin_LIBRARIES})

install(TARGETS keyboard_control pub_velosity
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)