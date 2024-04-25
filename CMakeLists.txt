cmake_minimum_required(VERSION 3.8)
project(purepursuit)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(purepursuit src/purepursuit.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(
  purepursuit
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  purepursuit
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# This must be at the bottom
ament_package()