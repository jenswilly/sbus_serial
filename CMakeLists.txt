cmake_minimum_required(VERSION 3.8)
project(sbus_serial)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find and dependencies for all node executables
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sbus_interfaces REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "sbus_interfaces")

# Add executable and link dependencies
add_executable(sbus_serial src/sbus_serial_node.cpp src/sbus_serial_driver.cpp)
ament_target_dependencies(sbus_serial ${dependencies})

add_executable(sbus_test_publisher src/sbus_test_publisher.cpp)
ament_target_dependencies(sbus_test_publisher ${dependencies})

add_executable(sbus_cmd_vel src/sbus_cmd_vel_node.cpp)
ament_target_dependencies(sbus_cmd_vel ${dependencies})

# Separate executable for the calibration
add_executable(sbus_calibrate src/sbus_calibrate.cpp src/sbus_serial_driver.cpp)

# Install executables in lib
install(
	TARGETS 
		sbus_serial 
		sbus_test_publisher
		sbus_cmd_vel
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install calibration binary in share
install(
	TARGETS sbus_calibrate
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
