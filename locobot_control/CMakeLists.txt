cmake_minimum_required(VERSION 3.5)
project(locobot_control)

## Compile as C++14 (ROS2 default?)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(LIB_NAME "locobot_controller")

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(simple_actions REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Setup for python modules and scripts
################################################################################
# TODO: how to do this in ROS 2?
#catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JointCommand.srv"
)

# No pinocchio for now
#SET(PINOCCHIO_DIR $ENV{PINOCCHIO_PATH} CACHE PATH "Path to pinocchio dir")
#include_directories(${PINOCCHIO_DIR}/include)
#find_package(pinocchio REQUIRED HINTS ${PINOCCHIO_DIR}/lib)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${LIB_NAME} src/locobot_controller.cpp)
ament_target_dependencies(${LIB_NAME}
  rclcpp
  rclcpp_action
  simple_actions
  control_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  std_srvs
  #pinocchio::pinocchio
)
target_link_libraries(${LIB_NAME} ${YAML_CPP_LIBRARIES}) 
# Link message/service/action defined in the same cmakelists
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${LIB_NAME} "${cpp_typesupport_target}") 


target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${YAML_CPP_LIBRARIES}
)

#################################################################################
## Install
#################################################################################
# install(TARGETS locobot_control
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

#ament_export_dependencies(dynamixel_sdk)
#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

