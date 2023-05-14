cmake_minimum_required(VERSION 3.1)

set(CEPTON2_ROS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")

project(cepton2_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  #pluginlib
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SensorInformation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CEPTON2_ROS_LIBRARIES "")

add_library(
  cepton2_ros
  "${PROJECT_SOURCE_DIR}/src/publisher_nodelet.cpp"
  "${PROJECT_SOURCE_DIR}/src/common.cpp"
)
list(APPEND CEPTON2_ROS_LIBRARIES cepton2_ros)

message(STATUS "Detected processor: ${CMAKE_SYSTEM_PROCESSOR}")
foreach(name IN LISTS CEPTON2_ROS_LIBRARIES)
  target_include_directories(
    ${name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
    target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/x86_64/libcepton_sdk2.so)
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
    target_link_libraries(${name} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/aarch64/libcepton_sdk2.so)
  else()
    message("NO LIBRARY FOUND MATCHING PROCESSOR")
  endif()
  add_dependencies(${name} cepton2_ros_generate_messages_cpp)
endforeach()

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${CEPTON2_ROS_LIBRARIES}
CATKIN_DEPENDS message_runtime nodelet pcl_conversions pcl_ros roscpp pluginlib roslib std_msgs
#DEPENDS system_lib
)

install(DIRECTORY "include/cepton2_ros/"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS ${CEPTON2_ROS_LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
