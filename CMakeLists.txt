################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

add_library(${PROJECT_NAME}_lib
  "src/turtlebot3.cpp"

  "src/sensors/battery_state.cpp"
  "src/sensors/joint_state.cpp"
  "src/sensors/AdcReader.cpp"
  "src/sensors/wheel_frequency_reader.cpp"
  "src/sensors/wheel_command_state.cpp"

  "src/devices/motor_pwm_dir.cpp"
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  pigpio
  message_filters::message_filters
  rclcpp::rclcpp
  rcutils::rcutils
  tf2::tf2
  tf2_ros::tf2_ros
)

ament_export_dependencies(
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
