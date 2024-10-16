cmake_minimum_required(VERSION 2.8.3)
project(robotic_arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_commander
  moveit_ros_planning_interface
  teleop_twist_keyboard
  moveit_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotic_arm_control
  CATKIN_DEPENDS roscpp std_msgs moveit_commander moveit_ros_planning_interface teleop_twist_keyboard moveit_core
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
target_link_libraries(inverse_kinematics
  ${catkin_LIBRARIES}
)

install(TARGETS inverse_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(inverse_kinematics ${PROJECT_NAME})