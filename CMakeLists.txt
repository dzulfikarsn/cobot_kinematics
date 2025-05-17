cmake_minimum_required(VERSION 3.8)
project(cobot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)
find_package(tf2)
find_package(tf2_ros)

# ========== FK NODE ==========
# Executable node
add_executable(fk_node
  src/fk_node.cpp
  src/fk.cpp
)

# Modern include path handling
target_include_directories(fk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS 2 and external dependencies
ament_target_dependencies(fk_node
  kdl_parser
  orocos_kdl
  sensor_msgs
  rclcpp
)
# =============================

# ========== IK NODE ==========
add_executable(ik_node
  src/ik_node.cpp
  src/ik.cpp
)

target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_node
  geometry_msgs
  kdl_parser
  orocos_kdl
  rclcpp
  tf2
  tf2_ros
)
# =============================

# Install target
install(TARGETS
  fk_node
  ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
