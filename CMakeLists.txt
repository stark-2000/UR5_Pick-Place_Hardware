cmake_minimum_required(VERSION 3.0.2)
project(ur5v1)

find_package(catkin REQUIRED COMPONENTS
moveit_core
moveit_visual_tools
moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
    roscpp std_msgs
    tf_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_place_moveit_joint_control ./src/pick_place_moveit_joint_control.cpp)
target_link_libraries(pick_place_moveit_joint_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
