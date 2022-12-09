cmake_minimum_required(VERSION 2.8.3)
project(ur5v1_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    DEPENDS
    numcpp
)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-pg")
add_compile_options(-std=c++14)

set(UR5V1_IK_LIBRARY_NAME ur5v1_ik_kinematics_plugin)

add_library(${UR5V1_IK_LIBRARY_NAME} src/ur5v1_ik_kinematics_plugin.cpp src/dhparam.cpp)

target_link_libraries(${UR5V1_IK_LIBRARY_NAME} ${catkin_LIBRARIES})

install(TARGETS ${UR5V1_IK_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
    ur5v1_ik_kinematics_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
